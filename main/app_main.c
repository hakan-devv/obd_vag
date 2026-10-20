#include "can_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "obd_client.h"
#include "onx_display.h"
#include "sdkconfig.h"

#if !defined(CONFIG_OBD_RUN_PROTOCOL_TESTS) || !CONFIG_OBD_RUN_PROTOCOL_TESTS
static const char *TAG = "app_main";
#endif

void app_main(void)
{
#if defined(CONFIG_OBD_RUN_PROTOCOL_TESTS) && CONFIG_OBD_RUN_PROTOCOL_TESTS
    obd_client_run_protocol_tests();
    return;
#else
    esp_err_t err = can_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "can_init failed: %s", esp_err_to_name(err));
        return;
    }

    /* LCD is optional for CAN bring-up; keep CAN diagnostics running if display init fails. */
    err = onx_display_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ONX LCD UI unavailable; continuing CAN diagnostics without display updates");
    }

#if defined(CONFIG_OBD_RUN_CAN_SELF_TEST) && CONFIG_OBD_RUN_CAN_SELF_TEST
    /* Internal loopback; the test frame does not reach the physical CAN bus. */
    err = can_run_self_test();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "can_run_self_test failed: %s", esp_err_to_name(err));
        return;
    }
#else
    ESP_LOGI(TAG, "MCP2515 loopback self-test skipped by configuration");
#endif

    err = can_set_mode(CAN_MODE_LISTEN_ONLY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not enter Listen-Only mode: %s", esp_err_to_name(err));
        return;
    }

    const can_frame_t guard_test = {
        .id = 0x123U, .dlc = 1U, .data = {0xA5U}, .extended = false,
    };
    err = can_send(&guard_test);
    if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Listen-Only TX guard FAILED: %s", esp_err_to_name(err));
        return;
    }
    can_diagnostics_t diag;
    err = can_read_diagnostics(&diag);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not inspect TX buffers after Listen-Only guard: %s",
                 esp_err_to_name(err));
        return;
    }
    if (diag.txreq_mask != 0) {
        ESP_LOGE(TAG, "Listen-Only TX guard FAILED at register level: TXREQ mask=0x%02X",
                 diag.txreq_mask);
        return;
    }
    ESP_LOGI(TAG, "Listen-Only TX guard passed; TXREQ clear on TXB0/TXB1/TXB2");

    /* Select passive Listen-Only or active OBD requests via Kconfig; passive is default. */
    err = obd_client_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "obd_client_start failed: %s", esp_err_to_name(err));
        return;
    }

#if defined(CONFIG_OBD_LOOPBACK_ECU_SIMULATOR) && CONFIG_OBD_LOOPBACK_ECU_SIMULATOR
    ESP_LOGI(TAG, "Internal OBD-II ECU simulator active; MCP2515 Loopback mode keeps traffic off the physical CAN bus");
#elif defined(CONFIG_OBD_ACTIVE_TX) && CONFIG_OBD_ACTIVE_TX
    ESP_LOGI(TAG, "Active OBD RPM requests ENABLED by build configuration");
#else
    ESP_LOGI(TAG, "Passive Listen-Only CAN monitor active; firmware will NOT transmit or ACK CAN frames");
#endif
    while (true) {
        obd_client_poll();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
#endif
}
