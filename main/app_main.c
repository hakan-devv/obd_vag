#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "can_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "obd_client.h"
#include "onx_display.h"
#include "sdkconfig.h"

static const char *TAG = "app_main";

/* Runtime command responses use the same USB Serial/JTAG console as ESP_LOGx. */
static void app_console_print(const char *message)
{
    if (message != NULL) {
        ESP_LOGI(TAG, "%s", message);
    }
}

static void app_handle_runtime_command(char command)
{
    char cmd = (char)tolower((unsigned char)command);
    switch (cmd) {
    case '0':
        obd_client_set_pid_polling_enabled(false);
        app_console_print("[PID] Polling STOPPED");
        break;
    case '1':
        obd_client_set_pid_polling_enabled(true);
        app_console_print("[PID] Polling STARTED");
        break;
    case '?':
        app_console_print(obd_client_is_pid_polling_enabled() ?
                       "[PID] Polling STATUS: ENABLED" :
                       "[PID] Polling STATUS: DISABLED");
        break;
    case 'p':
        obd_client_set_pid_polling_enabled(!obd_client_is_pid_polling_enabled());
        app_console_print(obd_client_is_pid_polling_enabled() ?
                       "[PID] Polling STARTED" :
                       "[PID] Polling STOPPED");
        break;
    case 'v':
        if (obd_client_vehicle_identify() == ESP_OK) {
            app_console_print("[VEH] Vehicle identification started");
        } else {
            app_console_print("[VEH] Vehicle identification rejected");
        }
        break;
    case 'd': {
        if (obd_client_did_scan_powertrain_start() == ESP_OK) {
            app_console_print("[DID] Broad engine + TCU candidate scan started (Haldex route is not enabled)");
        } else {
            app_console_print("[DID] Broad powertrain DID scan not started");
        }
        break;
    }
    default:
        break;
    }
}

static void app_console_command_task(void *arg)
{
    (void)arg;

    while (true) {
        const int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* IDF monitor commonly sends CRLF; commands are single characters. */
        if (ch == '\r' || ch == '\n') {
            continue;
        }

        app_handle_runtime_command((char)ch);
    }
}

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

    ESP_LOGI(TAG, "Automatic vehicle identification and EXACT-only OEM DID gating enabled");

    /*
     * Read commands from stdin in a dedicated task. With USB Serial/JTAG selected
     * as the primary ESP-IDF console, stdin and ESP_LOGx share the native USB
     * connection. A dedicated task is important: getchar() may block while the
     * OBD/CAN polling loop below must continue running.
     */
    setvbuf(stdin, NULL, _IONBF, 0);
    BaseType_t task_result = xTaskCreate(app_console_command_task,
                                         "console_cmd",
                                         3072,
                                         NULL,
                                         5,
                                         NULL);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Could not create USB console command task");
        return;
    }

    ESP_LOGI(TAG, "USB Serial/JTAG command console ready: 0/1/?/p/v/d");
    ESP_LOGI(TAG, "[PID] Polling STATUS: %s",
             obd_client_is_pid_polling_enabled() ? "ENABLED" : "DISABLED");

    while (true) {
        obd_client_poll();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
#endif
}
