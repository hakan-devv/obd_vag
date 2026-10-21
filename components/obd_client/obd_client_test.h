#ifndef OBD_CLIENT_TEST_H
#define OBD_CLIENT_TEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool obd_client_test_process_response(uint8_t requested_pid,
                                      const uint8_t *payload, size_t length);
bool obd_client_test_expire_request(uint8_t requested_pid, int64_t elapsed_us);

#endif
