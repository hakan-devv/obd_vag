#include "isotp.h"

#include <string.h>

#define ISOTP_RX_TIMEOUT_US       1000000ULL
#define ISOTP_MAX_WAIT_FRAMES     3U
#define ISOTP_FLOW_CONTROL_LEN    3U

static void set_event(isotp_event_t *event, isotp_event_type_t type,
                      isotp_error_t error)
{
    if (event == NULL) return;
    event->type = type;
    event->error = error;
    event->length = 0;
}

static esp_err_t send_padded_frame(isotp_context_t *context, const uint8_t *data,
                                   uint8_t length)
{
    can_frame_t frame = {
        .id = context->config.tx_id,
        .dlc = 8U,
        .extended = context->config.extended_id,
        .rtr = false,
        .data = {0},
    };
    memset(frame.data, context->config.padding, sizeof(frame.data));
    memcpy(frame.data, data, length);
    return context->send_frame(&frame, context->send_context);
}

static esp_err_t send_flow_control(isotp_context_t *context)
{
    const uint8_t flow_control[ISOTP_FLOW_CONTROL_LEN] = {
        0x30U, context->config.rx_block_size, context->config.rx_st_min,
    };
    return send_padded_frame(context, flow_control, sizeof(flow_control));
}

static uint64_t st_min_to_us(uint8_t st_min)
{
    if (st_min <= 0x7FU) return (uint64_t)st_min * 1000ULL;
    if (st_min >= 0xF1U && st_min <= 0xF9U) {
        return (uint64_t)(st_min - 0xF0U) * 100ULL;
    }
    return UINT64_MAX;
}

esp_err_t isotp_init(isotp_context_t *context, const isotp_config_t *config,
                     isotp_send_frame_fn send_frame, void *send_context)
{
    if (context == NULL || config == NULL || send_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((!config->extended_id && (config->tx_id > 0x7FFU || config->rx_id > 0x7FFU)) ||
        (config->extended_id && (config->tx_id > 0x1FFFFFFFU ||
                                 config->rx_id > 0x1FFFFFFFU))) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(context, 0, sizeof(*context));
    context->config = *config;
    context->send_frame = send_frame;
    context->send_context = send_context;
    return ESP_OK;
}

void isotp_reset(isotp_context_t *context)
{
    if (context == NULL) return;
    context->tx_length = 0;
    context->tx_offset = 0;
    context->rx_length = 0;
    context->rx_offset = 0;
    context->deadline_us = 0;
    context->next_tx_us = 0;
    context->tx_sequence = 0;
    context->rx_sequence = 0;
    context->tx_block_size = 0;
    context->tx_block_count = 0;
    context->rx_block_count = 0;
    context->st_min = 0;
    context->wait_frames = 0;
    context->tx_waiting_flow_control = false;
    context->rx_active = false;
    context->tx_active = false;
}

esp_err_t isotp_send(isotp_context_t *context, const uint8_t *payload,
                     size_t length, uint64_t now_us)
{
    if (context == NULL || payload == NULL || length == 0U ||
        length > ISOTP_MAX_PAYLOAD) return ESP_ERR_INVALID_ARG;
    if (context->tx_active) return ESP_ERR_INVALID_STATE;

    if (length <= 7U) {
        uint8_t single_frame[8] = {0};
        single_frame[0] = (uint8_t)length;
        memcpy(&single_frame[1], payload, length);
        return send_padded_frame(context, single_frame, (uint8_t)(length + 1U));
    }

    context->tx_length = (uint16_t)length;
    context->tx_offset = 6U;
    context->tx_sequence = 1U;
    context->tx_block_count = 0U;
    context->wait_frames = 0U;
    context->tx_waiting_flow_control = true;
    context->tx_active = true;
    context->deadline_us = now_us + ISOTP_RX_TIMEOUT_US;
    memcpy(context->tx_data, payload, length);

    const uint8_t first_frame[8] = {
        (uint8_t)(0x10U | ((length >> 8) & 0x0FU)),
        (uint8_t)length,
        payload[0], payload[1], payload[2], payload[3], payload[4], payload[5],
    };
    esp_err_t err = send_padded_frame(context, first_frame, sizeof(first_frame));
    if (err != ESP_OK) {
        context->tx_active = false;
        return err;
    }
    return ESP_OK;
}

static isotp_event_type_t handle_flow_control(isotp_context_t *context,
                                              const can_frame_t *frame,
                                              uint64_t now_us,
                                              isotp_event_t *event)
{
    if (!context->tx_active || !context->tx_waiting_flow_control) return ISOTP_EVENT_NONE;
    if (frame->dlc < 3U) {
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_BAD_FRAME);
        context->tx_active = false;
        return ISOTP_EVENT_ERROR;
    }

    const uint8_t flow_status = frame->data[0] & 0x0FU;
    if (flow_status == 0x01U) {
        if (++context->wait_frames > ISOTP_MAX_WAIT_FRAMES) {
            set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_FLOW_CONTROL);
            context->tx_active = false;
            return ISOTP_EVENT_ERROR;
        }
        context->deadline_us = now_us + ISOTP_RX_TIMEOUT_US;
        return ISOTP_EVENT_NONE;
    }
    if (flow_status != 0x00U) {
        set_event(event, ISOTP_EVENT_ERROR,
                  flow_status == 0x02U ? ISOTP_ERROR_OVERFLOW : ISOTP_ERROR_FLOW_CONTROL);
        context->tx_active = false;
        return ISOTP_EVENT_ERROR;
    }

    context->tx_block_size = frame->data[1];
    context->st_min = frame->data[2];
    if (st_min_to_us(context->st_min) == UINT64_MAX) {
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_FLOW_CONTROL);
        context->tx_active = false;
        return ISOTP_EVENT_ERROR;
    }
    context->tx_waiting_flow_control = false;
    context->wait_frames = 0;
    context->tx_block_count = 0;
    context->next_tx_us = now_us + st_min_to_us(context->st_min);
    context->deadline_us = now_us + ISOTP_RX_TIMEOUT_US;
    return ISOTP_EVENT_NONE;
}

static isotp_event_type_t handle_single_frame(isotp_context_t *context,
                                              const can_frame_t *frame,
                                              isotp_event_t *event)
{
    const uint8_t length = frame->data[0] & 0x0FU;
    if (length == 0U || length > 7U || (uint16_t)(length + 1U) > frame->dlc) {
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_BAD_FRAME);
        return ISOTP_EVENT_ERROR;
    }
    set_event(event, ISOTP_EVENT_PDU_RECEIVED, ISOTP_ERROR_NONE);
    if (event != NULL) {
        event->length = length;
        memcpy(event->data, &frame->data[1], length);
    }
    return ISOTP_EVENT_PDU_RECEIVED;
}

static isotp_event_type_t handle_first_frame(isotp_context_t *context,
                                             const can_frame_t *frame,
                                             uint64_t now_us,
                                             isotp_event_t *event)
{
    const uint16_t length = (uint16_t)(((uint16_t)(frame->data[0] & 0x0FU) << 8) |
                                       frame->data[1]);
    if (frame->dlc < 8U || length <= 7U || length > ISOTP_MAX_PAYLOAD) {
        set_event(event, ISOTP_EVENT_ERROR,
                  length > ISOTP_MAX_PAYLOAD ? ISOTP_ERROR_OVERFLOW : ISOTP_ERROR_BAD_FRAME);
        context->rx_active = false;
        return ISOTP_EVENT_ERROR;
    }

    context->rx_length = length;
    context->rx_offset = 6U;
    context->rx_sequence = 1U;
    context->rx_block_count = 0U;
    context->rx_active = true;
    context->deadline_us = now_us + ISOTP_RX_TIMEOUT_US;
    memcpy(context->rx_data, &frame->data[2], 6U);

    esp_err_t err = send_flow_control(context);
    if (err != ESP_OK) {
        context->rx_active = false;
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_FLOW_CONTROL);
        return ISOTP_EVENT_ERROR;
    }
    return ISOTP_EVENT_NONE;
}

static isotp_event_type_t handle_consecutive_frame(isotp_context_t *context,
                                                   const can_frame_t *frame,
                                                   uint64_t now_us,
                                                   isotp_event_t *event)
{
    if (!context->rx_active || frame->dlc < 2U) {
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_BAD_FRAME);
        return ISOTP_EVENT_ERROR;
    }
    const uint8_t sequence = frame->data[0] & 0x0FU;
    if (sequence != context->rx_sequence) {
        context->rx_active = false;
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_SEQUENCE);
        return ISOTP_EVENT_ERROR;
    }

    const uint16_t remaining = context->rx_length - context->rx_offset;
    const uint8_t available = (uint8_t)(frame->dlc - 1U);
    const uint8_t copy_length = (uint8_t)(remaining < available ? remaining : available);
    memcpy(&context->rx_data[context->rx_offset], &frame->data[1], copy_length);
    context->rx_offset = (uint16_t)(context->rx_offset + copy_length);
    context->rx_sequence = (uint8_t)((context->rx_sequence + 1U) & 0x0FU);
    ++context->rx_block_count;
    context->deadline_us = now_us + ISOTP_RX_TIMEOUT_US;

    if (context->rx_offset == context->rx_length) {
        context->rx_active = false;
        set_event(event, ISOTP_EVENT_PDU_RECEIVED, ISOTP_ERROR_NONE);
        if (event != NULL) {
            event->length = context->rx_length;
            memcpy(event->data, context->rx_data, context->rx_length);
        }
        return ISOTP_EVENT_PDU_RECEIVED;
    }

    if (context->config.rx_block_size != 0U &&
        context->rx_block_count >= context->config.rx_block_size) {
        context->rx_block_count = 0U;
        if (send_flow_control(context) != ESP_OK) {
            context->rx_active = false;
            set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_FLOW_CONTROL);
            return ISOTP_EVENT_ERROR;
        }
    }
    return ISOTP_EVENT_NONE;
}

isotp_event_type_t isotp_on_frame(isotp_context_t *context,
                                  const can_frame_t *frame, uint64_t now_us,
                                  isotp_event_t *event)
{
    if (context == NULL || frame == NULL || event == NULL) return ISOTP_EVENT_ERROR;
    set_event(event, ISOTP_EVENT_NONE, ISOTP_ERROR_NONE);
    if (frame->id != context->config.tx_id && frame->id != context->config.rx_id) {
        return ISOTP_EVENT_NONE;
    }
    if (frame->extended != context->config.extended_id || frame->rtr || frame->dlc == 0U) {
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_BAD_FRAME);
        return ISOTP_EVENT_ERROR;
    }

    if (frame->id == context->config.rx_id && (frame->data[0] >> 4) == 0x03U) {
        return handle_flow_control(context, frame, now_us, event);
    }
    if (frame->id != context->config.rx_id) return ISOTP_EVENT_NONE;

    switch (frame->data[0] >> 4) {
    case 0x00U:
        return handle_single_frame(context, frame, event);
    case 0x01U:
        return handle_first_frame(context, frame, now_us, event);
    case 0x02U:
        return handle_consecutive_frame(context, frame, now_us, event);
    default:
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_BAD_FRAME);
        return ISOTP_EVENT_ERROR;
    }
}

isotp_event_type_t isotp_tick(isotp_context_t *context, uint64_t now_us,
                              isotp_event_t *event)
{
    if (context == NULL || event == NULL) return ISOTP_EVENT_ERROR;
    set_event(event, ISOTP_EVENT_NONE, ISOTP_ERROR_NONE);

    if ((context->rx_active || context->tx_active) && now_us >= context->deadline_us) {
        const isotp_error_t error = context->rx_active
            ? ISOTP_ERROR_RX_TIMEOUT : ISOTP_ERROR_TX_TIMEOUT;
        context->rx_active = false;
        context->tx_active = false;
        context->tx_waiting_flow_control = false;
        set_event(event, ISOTP_EVENT_ERROR, error);
        return ISOTP_EVENT_ERROR;
    }

    if (!context->tx_active || context->tx_waiting_flow_control ||
        now_us < context->next_tx_us) return ISOTP_EVENT_NONE;

    const uint16_t remaining = context->tx_length - context->tx_offset;
    const uint8_t copy_length = (uint8_t)(remaining > 7U ? 7U : remaining);
    uint8_t consecutive_frame[8] = {0};
    consecutive_frame[0] = (uint8_t)(0x20U | (context->tx_sequence & 0x0FU));
    memcpy(&consecutive_frame[1], &context->tx_data[context->tx_offset], copy_length);
    esp_err_t err = send_padded_frame(context, consecutive_frame,
                                      (uint8_t)(copy_length + 1U));
    if (err != ESP_OK) {
        context->tx_active = false;
        set_event(event, ISOTP_EVENT_ERROR, ISOTP_ERROR_FLOW_CONTROL);
        return ISOTP_EVENT_ERROR;
    }

    context->tx_offset = (uint16_t)(context->tx_offset + copy_length);
    context->tx_sequence = (uint8_t)((context->tx_sequence + 1U) & 0x0FU);
    ++context->tx_block_count;
    context->deadline_us = now_us + ISOTP_RX_TIMEOUT_US;

    if (context->tx_offset == context->tx_length) {
        context->tx_active = false;
        set_event(event, ISOTP_EVENT_TX_COMPLETE, ISOTP_ERROR_NONE);
        return ISOTP_EVENT_TX_COMPLETE;
    }

    if (context->tx_block_size != 0U &&
        context->tx_block_count >= context->tx_block_size) {
        context->tx_waiting_flow_control = true;
    } else {
        context->next_tx_us = now_us + st_min_to_us(context->st_min);
    }
    return ISOTP_EVENT_NONE;
}
