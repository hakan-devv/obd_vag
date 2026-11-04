#ifndef OBD_CLIENT_H
#define OBD_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OBD_PID_ERROR_NONE = 0,
    OBD_PID_ERROR_TIMEOUT,
    OBD_PID_ERROR_NEGATIVE_RESPONSE,
    OBD_PID_ERROR_INVALID_RESPONSE,
    OBD_PID_ERROR_TX,
} obd_pid_error_t;

typedef enum {
    OBD_VEHICLE_MATCH_UNKNOWN = 0,
    OBD_VEHICLE_MATCH_PARTIAL,
    OBD_VEHICLE_MATCH_EXACT,
} obd_vehicle_match_t;

typedef struct {
    int64_t last_request_us;
    int64_t last_success_us;
    uint32_t error_count;
    obd_pid_error_t last_error;
} obd_pid_status_t;

typedef struct {
    bool initialized;
    bool mode09_support_known;
    bool vin_known;
    bool year_known;
    bool exact_ecu_match;
    bool auto_did_allowed;
    obd_vehicle_match_t match_level;
    uint32_t request_tx_id;
    uint32_t response_rx_id;
    uint32_t ecu_tx_id;
    uint32_t ecu_rx_id;
    char vin[18];
    char model_year[8];
    char ecu_profile[32];
    char mode09_support_bitmap[32];
    char calibration_id[32];
    char cvn[32];
    char f187[32];
    char f189[32];
    char f191[32];
    char f1a3[32];
    char f197[32];
    char f1ad[32];
    char f19e[32];
    char f1a2[32];
    char f1aa[32];
    char f17c[32];
    char f1f4[32];
    char f1a5[32];
} obd_vehicle_identity_t;

/* Starts the configured passive or active OBD-II monitor. */
esp_err_t obd_client_start(void);

/* Runtime control for the periodic Mode 01 PID scheduler without disabling CAN RX/UDS discovery. */
void obd_client_set_pid_polling_enabled(bool enabled);
bool obd_client_is_pid_polling_enabled(void);

/* Non-blocking service step: drains CAN RX and runs enabled diagnostic schedulers. */
void obd_client_poll(void);
bool obd_client_get_pid_status(uint8_t pid, obd_pid_status_t *status);
void obd_client_run_protocol_tests(void);
esp_err_t obd_client_vehicle_identify(void);
const obd_vehicle_identity_t *obd_client_vehicle_identity(void);
esp_err_t obd_client_uds_start_session(uint8_t ecu_index);
esp_err_t obd_client_uds_tester_present(uint8_t ecu_index);
esp_err_t obd_client_uds_read_did(uint8_t ecu_index, uint16_t did);
esp_err_t obd_client_did_scan_start(uint8_t ecu_index, const char *profile_id);
/* Broad cross-profile scan: remaining engine/identity candidates, then the configured 11-bit TCU route. */
esp_err_t obd_client_did_scan_powertrain_start(void);
void obd_client_did_scan_stop(void);
void obd_client_did_scan_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* OBD_CLIENT_H */
