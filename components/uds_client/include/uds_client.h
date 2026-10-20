#ifndef UDS_CLIENT_H
#define UDS_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "isotp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UDS_CLIENT_MAX_DATA ISOTP_MAX_PAYLOAD

typedef enum {
    UDS_SESSION_UNKNOWN = 0,
    UDS_SESSION_DEFAULT,
    UDS_SESSION_EXTENDED,
} uds_session_t;

typedef enum {
    UDS_EVENT_NONE = 0,
    UDS_EVENT_RESPONSE,
    UDS_EVENT_NEGATIVE_RESPONSE,
    UDS_EVENT_RESPONSE_PENDING,
    UDS_EVENT_ERROR,
} uds_event_type_t;

typedef enum {
    UDS_ERROR_NONE = 0,
    UDS_ERROR_TRANSPORT,
    UDS_ERROR_TIMEOUT,
    UDS_ERROR_BAD_RESPONSE,
} uds_error_t;

typedef struct {
    uds_event_type_t type;
    uds_error_t error;
    uint8_t service;
    uint8_t nrc;
    uint16_t did;
    uint16_t data_length;
    uint8_t data[UDS_CLIENT_MAX_DATA];
} uds_event_t;

typedef struct {
    isotp_context_t transport;
    isotp_event_t transport_event;
    uds_session_t session;
    uint8_t active_service;
    uint8_t active_subfunction;
    uint16_t active_did;
    uint64_t deadline_us;
    uint64_t response_timeout_us;
    uint64_t pending_timeout_us;
    bool transaction_active;
} uds_client_t;

esp_err_t uds_client_init(uds_client_t *client, const isotp_config_t *transport,
                          isotp_send_frame_fn send_frame, void *send_context);
esp_err_t uds_client_start_extended_session(uds_client_t *client,
                                            uint64_t now_us);
esp_err_t uds_client_tester_present(uds_client_t *client, uint64_t now_us);
esp_err_t uds_client_read_did(uds_client_t *client, uint16_t did,
                              uint64_t now_us);
uds_event_type_t uds_client_on_frame(uds_client_t *client,
                                    const can_frame_t *frame, uint64_t now_us,
                                    uds_event_t *event);
uds_event_type_t uds_client_tick(uds_client_t *client, uint64_t now_us,
                                 uds_event_t *event);
bool uds_client_transaction_active(const uds_client_t *client);

#ifdef __cplusplus
}
#endif

#endif
