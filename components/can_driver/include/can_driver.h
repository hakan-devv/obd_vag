#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool extended;
    bool rtr;
} can_frame_t;

typedef struct {
    uint8_t canstat;
    uint8_t eflg;
    uint8_t tec;
    uint8_t rec;
    /* bit 0/1/2 correspond to TXB0/TXB1/TXB2 TXREQ being set */
    uint8_t txreq_mask;
} can_diagnostics_t;

typedef enum {
    CAN_MODE_NORMAL = 0,
    CAN_MODE_LISTEN_ONLY,
    CAN_MODE_LOOPBACK,
    CAN_MODE_CONFIG,
} can_mode_t;

typedef enum {
    CAN_BITRATE_250_KBPS = 250000U,
    CAN_BITRATE_500_KBPS = 500000U,
} can_bitrate_t;

typedef struct {
    can_bitrate_t bitrate;
    bool extended_ids;
} can_bus_config_t;

esp_err_t can_init(void);
esp_err_t can_init_with_config(const can_bus_config_t *config);
/* Reconfiguration is allowed only in Configuration mode. */
esp_err_t can_reconfigure(const can_bus_config_t *config);
esp_err_t can_get_config(can_bus_config_t *config);
esp_err_t can_send(const can_frame_t *frame);
esp_err_t can_receive(can_frame_t *frame);
/* Returns ESP_OK and writes *available, or returns an SPI/state error. */
esp_err_t can_available(bool *available);
esp_err_t can_set_mode(can_mode_t mode);
esp_err_t can_read_diagnostics(can_diagnostics_t *diagnostics);
/* Read latched RX overflow flags and clear the observed flags; output is valid on ESP_OK. */
esp_err_t can_read_and_clear_rx_overflow(uint8_t *overflow_flags);
/* Configure standard OBD-II response IDs 0x7E8-0x7EF; requires Configuration mode. */
esp_err_t can_configure_obd_response_filters(void);
/* TEMPORARY diagnostics: accept all valid standard/extended IDs; requires Configuration mode. */
esp_err_t can_configure_diagnostic_accept_all_rx(void);
esp_err_t can_run_self_test(void);

#ifdef __cplusplus
}
#endif

#endif
