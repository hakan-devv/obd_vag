#ifndef ONX_DISPLAY_H
#define ONX_DISPLAY_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Two-page UI for the ONX2432G028 onboard ST7789 LCD and CST826 touch. */
esp_err_t onx_display_init(void);
void onx_display_set_rpm(uint32_t rpm);
void onx_display_clear_rpm(void);
void onx_display_set_coolant_temp(int32_t temp_c);
void onx_display_clear_coolant_temp(void);
void onx_display_set_status(const char *status);
void onx_display_show_waiting_for_rpm(const char *status);

#ifdef __cplusplus
}
#endif

#endif /* ONX_DISPLAY_H */
