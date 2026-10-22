#include "uds_client.h"

#include <string.h>

#define UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL 0x10U
#define UDS_SERVICE_TESTER_PRESENT              0x3EU
#define UDS_SERVICE_READ_DATA_BY_IDENTIFIER     0x22U
#define UDS_NRC_RESPONSE_PENDING                0x78U
#define UDS_DEFAULT_RESPONSE_TIMEOUT_US         1000000ULL
#define UDS_PENDING_RESPONSE_TIMEOUT_US         5000000ULL

static void clear_event(uds_event_t *event)
{
    if (event != NULL) {
        memset(event, 0, sizeof(*event));
        event->type = UDS_EVENT_NONE;
    }
}

static void finish_transaction(uds_client_t *client)
{
    client->transaction_active = false;
    client->active_service = 0U;
    client->active_subfunction = 0U;
    client->active_did = 0U;
    client->deadline_us = 0U;
}

esp_err_t uds_client_init(uds_client_t *client, const isotp_config_t *transport,
                          isotp_send_frame_fn send_frame, void *send_context)
{
    if (client == NULL || transport == NULL || send_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(client, 0, sizeof(*client));
    esp_err_t err = isotp_init(&client->transport, transport, send_frame,
                               send_context);
    if (err != ESP_OK) return err;

    client->session = UDS_SESSION_UNKNOWN;
    client->response_timeout_us = UDS_DEFAULT_RESPONSE_TIMEOUT_US;
    client->pending_timeout_us = UDS_PENDING_RESPONSE_TIMEOUT_US;
    return ESP_OK;
}

static esp_err_t start_request(uds_client_t *client, uint8_t service,
                               uint8_t subfunction, uint16_t did,
                               const uint8_t *request, size_t request_length,
                               uint64_t now_us)
{
    if (client == NULL || request == NULL || request_length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (client->transaction_active) return ESP_ERR_INVALID_STATE;

    esp_err_t err = isotp_send(&client->transport, request, request_length, now_us);
    if (err != ESP_OK) return err;

    client->active_service = service;
    client->active_subfunction = subfunction;
    client->active_did = did;
    client->deadline_us = now_us + client->response_timeout_us;
    client->transaction_active = true;
    return ESP_OK;
}

esp_err_t uds_client_start_extended_session(uds_client_t *client,
                                            uint64_t now_us)
{
    const uint8_t request[2] = {
        UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL, 0x03U,
    };
    return start_request(client, UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL, 0x03U,
                         0U, request, sizeof(request), now_us);
}

esp_err_t uds_client_tester_present(uds_client_t *client, uint64_t now_us)
{
    const uint8_t request[2] = {UDS_SERVICE_TESTER_PRESENT, 0x00U};
    return start_request(client, UDS_SERVICE_TESTER_PRESENT, 0x00U, 0U,
                         request, sizeof(request), now_us);
}

esp_err_t uds_client_read_did(uds_client_t *client, uint16_t did,
                              uint64_t now_us)
{
    const uint8_t request[3] = {
        UDS_SERVICE_READ_DATA_BY_IDENTIFIER,
        (uint8_t)(did >> 8),
        (uint8_t)did,
    };
    return start_request(client, UDS_SERVICE_READ_DATA_BY_IDENTIFIER, 0U, did,
                         request, sizeof(request), now_us);
}

static uds_event_type_t fail_response(uds_client_t *client, uds_event_t *event,
                                      uds_error_t error)
{
    finish_transaction(client);
    if (event != NULL) {
        event->type = UDS_EVENT_ERROR;
        event->error = error;
    }
    return UDS_EVENT_ERROR;
}

static uds_event_type_t process_pdu(uds_client_t *client, const uint8_t *data,
                                    size_t length, uint64_t now_us,
                                    uds_event_t *event)
{
    if (!client->transaction_active) return UDS_EVENT_NONE;
    if (length == 0U) return fail_response(client, event, UDS_ERROR_BAD_RESPONSE);

    if (data[0] == 0x7FU) {
        if (length < 3U || data[1] != client->active_service) {
            return UDS_EVENT_NONE;
        }
        if (data[2] == UDS_NRC_RESPONSE_PENDING) {
            client->deadline_us = now_us + client->pending_timeout_us;
            if (event != NULL) {
                event->type = UDS_EVENT_RESPONSE_PENDING;
                event->service = client->active_service;
                event->did = client->active_did;
                event->nrc = data[2];
            }
            return UDS_EVENT_RESPONSE_PENDING;
        }
        if (event != NULL) {
            event->type = UDS_EVENT_NEGATIVE_RESPONSE;
            event->service = client->active_service;
            event->did = client->active_did;
            event->nrc = data[2];
        }
        finish_transaction(client);
        return UDS_EVENT_NEGATIVE_RESPONSE;
    }

    if (data[0] != (uint8_t)(client->active_service + 0x40U)) {
        return UDS_EVENT_NONE;
    }

    size_t header_length = 1U;
    if (client->active_service == UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL ||
        client->active_service == UDS_SERVICE_TESTER_PRESENT) {
        if (length < 2U ||
            (data[1] & 0x7FU) != client->active_subfunction) {
            return fail_response(client, event, UDS_ERROR_BAD_RESPONSE);
        }
        header_length = 2U;
        if (client->active_service == UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL) {
            client->session = client->active_subfunction == 0x03U
                ? UDS_SESSION_EXTENDED : UDS_SESSION_DEFAULT;
        }
    } else if (client->active_service == UDS_SERVICE_READ_DATA_BY_IDENTIFIER) {
        if (length < 3U ||
            (((uint16_t)data[1] << 8) | data[2]) != client->active_did) {
            return fail_response(client, event, UDS_ERROR_BAD_RESPONSE);
        }
        header_length = 3U;
    } else {
        return fail_response(client, event, UDS_ERROR_BAD_RESPONSE);
    }

    if (event != NULL) {
        event->type = UDS_EVENT_RESPONSE;
        event->service = client->active_service;
        event->did = client->active_did;
        event->data_length = (uint16_t)(length - header_length);
        if (event->data_length != 0U) {
            memcpy(event->data, &data[header_length], event->data_length);
        }
    }
    finish_transaction(client);
    return UDS_EVENT_RESPONSE;
}

uds_event_type_t uds_client_on_frame(uds_client_t *client,
                                    const can_frame_t *frame, uint64_t now_us,
                                    uds_event_t *event)
{
    if (client == NULL || frame == NULL || event == NULL) {
        return UDS_EVENT_ERROR;
    }
    clear_event(event);

    isotp_event_t *transport_event = &client->transport_event;
    const isotp_event_type_t result = isotp_on_frame(&client->transport, frame,
        now_us, transport_event);
    if (result == ISOTP_EVENT_ERROR) {
        finish_transaction(client);
        event->type = UDS_EVENT_ERROR;
        event->error = UDS_ERROR_TRANSPORT;
        return UDS_EVENT_ERROR;
    }
    if (result == ISOTP_EVENT_PDU_RECEIVED) {
        return process_pdu(client, transport_event->data, transport_event->length,
                           now_us, event);
    }
    return UDS_EVENT_NONE;
}

uds_event_type_t uds_client_tick(uds_client_t *client, uint64_t now_us,
                                 uds_event_t *event)
{
    if (client == NULL || event == NULL) return UDS_EVENT_ERROR;
    clear_event(event);

    isotp_event_t *transport_event = &client->transport_event;
    const isotp_event_type_t transport_result =
        isotp_tick(&client->transport, now_us, transport_event);
    if (transport_result == ISOTP_EVENT_ERROR) {
        finish_transaction(client);
        event->type = UDS_EVENT_ERROR;
        event->error = UDS_ERROR_TRANSPORT;
        return UDS_EVENT_ERROR;
    }
    if (client->transaction_active && now_us >= client->deadline_us) {
        finish_transaction(client);
        event->type = UDS_EVENT_ERROR;
        event->error = UDS_ERROR_TIMEOUT;
        return UDS_EVENT_ERROR;
    }
    return UDS_EVENT_NONE;
}

bool uds_client_transaction_active(const uds_client_t *client)
{
    return client != NULL && client->transaction_active;
}
