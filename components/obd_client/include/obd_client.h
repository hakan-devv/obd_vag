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

typedef struct {
    int64_t last_request_us;
    int64_t last_success_us;
    uint32_t error_count;
    obd_pid_error_t last_error;
} obd_pid_status_t;

/* Starts the configured passive or active OBD-II monitor. */
esp_err_t obd_client_start(void);

/* Non-blocking service step: drains CAN RX and runs enabled diagnostic schedulers. */
void obd_client_poll(void);
bool obd_client_get_pid_status(uint8_t pid, obd_pid_status_t *status);
void obd_client_run_protocol_tests(void);
esp_err_t obd_client_uds_start_session(uint8_t ecu_index);
esp_err_t obd_client_uds_tester_present(uint8_t ecu_index);
esp_err_t obd_client_uds_read_did(uint8_t ecu_index, uint16_t did);

#ifdef __cplusplus
}
#endif

#endif /* OBD_CLIENT_H */
