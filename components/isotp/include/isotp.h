#ifndef ISOTP_H
#define ISOTP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_driver.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ISOTP_MAX_PAYLOAD 1024U

typedef esp_err_t (*isotp_send_frame_fn)(const can_frame_t *frame, void *context);

typedef struct {
    uint32_t tx_id;
    uint32_t rx_id;
    bool extended_id;
    uint8_t padding;
    uint8_t rx_block_size;
    uint8_t rx_st_min;
} isotp_config_t;

typedef enum {
    ISOTP_EVENT_NONE = 0,
    ISOTP_EVENT_PDU_RECEIVED,
    ISOTP_EVENT_TX_COMPLETE,
    ISOTP_EVENT_ERROR,
} isotp_event_type_t;

typedef enum {
    ISOTP_ERROR_NONE = 0,
    ISOTP_ERROR_BAD_FRAME,
    ISOTP_ERROR_SEQUENCE,
    ISOTP_ERROR_RX_TIMEOUT,
    ISOTP_ERROR_TX_TIMEOUT,
    ISOTP_ERROR_FLOW_CONTROL,
    ISOTP_ERROR_OVERFLOW,
    ISOTP_ERROR_BUSY,
} isotp_error_t;

typedef struct {
    isotp_event_type_t type;
    isotp_error_t error;
    uint16_t length;
    uint8_t data[ISOTP_MAX_PAYLOAD];
} isotp_event_t;

typedef struct {
    isotp_config_t config;
    isotp_send_frame_fn send_frame;
    void *send_context;
    uint8_t tx_data[ISOTP_MAX_PAYLOAD];
    uint8_t rx_data[ISOTP_MAX_PAYLOAD];
    uint16_t tx_length;
    uint16_t tx_offset;
    uint16_t rx_length;
    uint16_t rx_offset;
    uint64_t deadline_us;
    uint64_t next_tx_us;
    uint8_t tx_sequence;
    uint8_t rx_sequence;
    uint8_t tx_block_size;
    uint8_t tx_block_count;
    uint8_t rx_block_count;
    uint8_t st_min;
    uint8_t wait_frames;
    bool tx_waiting_flow_control;
    bool rx_active;
    bool tx_active;
} isotp_context_t;

esp_err_t isotp_init(isotp_context_t *context, const isotp_config_t *config,
                     isotp_send_frame_fn send_frame, void *send_context);
esp_err_t isotp_send(isotp_context_t *context, const uint8_t *payload,
                     size_t length, uint64_t now_us);
isotp_event_type_t isotp_on_frame(isotp_context_t *context,
                                  const can_frame_t *frame, uint64_t now_us,
                                  isotp_event_t *event);
isotp_event_type_t isotp_tick(isotp_context_t *context, uint64_t now_us,
                              isotp_event_t *event);
void isotp_reset(isotp_context_t *context);

#ifdef __cplusplus
}
#endif

#endif
