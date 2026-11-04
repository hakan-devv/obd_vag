#ifndef ONX_DISPLAY_H
#define ONX_DISPLAY_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "parameter_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Three-value terminal-style pages for the onboard ST7789 LCD and CST826 touch. */
esp_err_t onx_display_init(void);
void onx_display_set_catalog(const parameter_descriptor_t *catalog, size_t count);
void onx_display_update_parameter(uint8_t pid, float value,
                                  parameter_validity_t validity,
                                  int64_t timestamp_us);
void onx_display_poll(int64_t now_us);
/* OEM UDS DID card. `state_text` NULL = live value; otherwise shown as the card status
 * (e.g. "NOT READY" for a supported DID without data yet). Cards are created on demand. */
void onx_display_update_did(uint16_t did, const char *name, const char *unit,
                            float value, const char *state_text,
                            uint32_t period_ms, int64_t timestamp_us);
void onx_display_set_status(const char *status);
void onx_display_show_waiting_for_rpm(const char *status);

#ifdef __cplusplus
}
#endif

#endif /* ONX_DISPLAY_H */
