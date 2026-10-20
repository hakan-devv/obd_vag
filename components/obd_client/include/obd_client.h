#ifndef OBD_CLIENT_H
#define OBD_CLIENT_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the configured passive or active OBD-II monitor. */
esp_err_t obd_client_start(void);

/* Non-blocking service step: drains CAN RX and runs enabled diagnostic schedulers. */
void obd_client_poll(void);
void obd_client_run_protocol_tests(void);
esp_err_t obd_client_uds_start_session(uint8_t ecu_index);
esp_err_t obd_client_uds_tester_present(uint8_t ecu_index);
esp_err_t obd_client_uds_read_did(uint8_t ecu_index, uint16_t did);

#ifdef __cplusplus
}
#endif

#endif /* OBD_CLIENT_H */
