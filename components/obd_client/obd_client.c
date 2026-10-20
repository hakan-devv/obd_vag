#include "obd_client.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "isotp.h"
#include "onx_display.h"
#include "parameter_manager.h"
#include "sdkconfig.h"
#include "uds_client.h"

static const char *TAG = "obd_client";

#define OBD_REQUEST_ID_STANDARD    0x7DFU
#define OBD_REQUEST_ID_EXTENDED    0x18DB33F1U
#define OBD_RESPONSE_ID_FIRST      0x7E8U
#define OBD_RESPONSE_ID_LAST       0x7EFU
#define OBD_MODE_CURRENT_DATA      0x01U
#define OBD_PID_COOLANT_TEMP       0x05U
#define OBD_PID_ENGINE_RPM         0x0CU
#define OBD_DISCOVERY_PID          0x00U
#define OBD_REQUEST_PERIOD_US        50000LL
#define OBD_RESPONSE_TIMEOUT_US     200000LL
#define OBD_MAX_RX_FRAMES_PER_STEP  32
#define OBD_MAX_CONSECUTIVE_MISSES  3U
#define OBD_RX_STATS_MAX_UNIQUE_IDS 32U
#define OBD_RX_OVERFLOW_POLL_US     50000LL
#define OBD_RX0_OVERFLOW_FLAG       0x40U
#define OBD_RX1_OVERFLOW_FLAG       0x80U
#define OBD_ECU_COUNT               8U
#define OBD_PID_PAGE_SIZE            0x20U
#define OBD_MAX_DISCOVERY_BASE       0xE0U
#define OBD_STALE_MULTIPLIER         3LL

#if defined(CONFIG_OBD_ACTIVE_TX) && CONFIG_OBD_ACTIVE_TX
#define OBD_ACTIVE_TX_ENABLED 1
#else
#define OBD_ACTIVE_TX_ENABLED 0
#endif

#if defined(CONFIG_OBD_LOOPBACK_ECU_SIMULATOR) && CONFIG_OBD_LOOPBACK_ECU_SIMULATOR
#define OBD_LOOPBACK_SIM_ENABLED 1
#else
#define OBD_LOOPBACK_SIM_ENABLED 0
#endif

#if OBD_ACTIVE_TX_ENABLED || OBD_LOOPBACK_SIM_ENABLED
#define OBD_SCHEDULER_ENABLED 1
#else
#define OBD_SCHEDULER_ENABLED 0
#endif

static bool s_started;
static bool s_polling_enabled;
static bool s_waiting_for_response;
static uint8_t s_requested_pid;
#if OBD_SCHEDULER_ENABLED
static int64_t s_next_request_us;
#endif
static int64_t s_request_sent_us;
static uint32_t s_consecutive_misses;
static uint32_t s_successful_rpm_responses;
static uint32_t s_successful_coolant_responses;
static int64_t s_last_rpm_response_us;
static bool s_rpm_displayed_live;
static int64_t s_last_coolant_response_us;
static bool s_coolant_displayed_live;
static uint32_t s_raw_frames_logged;
static int64_t s_last_diag_attempt_us;
static int64_t s_last_diag_success_us;
static int64_t s_last_rx_error_log_us;
static uint32_t s_rx_error_count;
static int64_t s_last_overflow_poll_us;
static int64_t s_last_overflow_log_us;
static int64_t s_last_overflow_error_log_us;
static uint32_t s_rx0_overflow_events;
static uint32_t s_rx1_overflow_events;
static uint32_t s_total_rx_frames;
static uint32_t s_rx_frames_since_diag;
static bool s_have_last_rx_frame;
static bool s_last_rx_extended;
static uint32_t s_last_rx_id;
static uint8_t s_last_rx_dlc;
static uint32_t s_unique_ids_count;
static bool s_unique_ids_table_full;
static can_bus_config_t s_can_config;
static isotp_context_t s_isotp[OBD_ECU_COUNT];
static uds_client_t s_uds[OBD_ECU_COUNT];
static isotp_event_t s_isotp_event;
static uds_event_t s_uds_event;
static uint8_t s_discovery_base;
static bool s_discovery_complete;
static bool s_discovery_response_seen;
static bool s_discovery_has_continuation;
static int64_t s_discovery_collect_until_us;
static int64_t s_next_due_us[256];
static uint32_t s_last_scheduler_lag_us;
static struct {
    uint32_t id;
    bool extended;
    bool used;
} s_unique_ids[OBD_RX_STATS_MAX_UNIQUE_IDS];

static void record_miss(const char *reason)
{
    s_waiting_for_response = false;
    s_requested_pid = 0U;
    onx_display_set_status("OBD REQUEST FAILED");
    ++s_consecutive_misses;
    ESP_LOGW(TAG, "%s (%lu/%u)", reason,
             (unsigned long)s_consecutive_misses,
             (unsigned)OBD_MAX_CONSECUTIVE_MISSES);

    if (s_consecutive_misses >= OBD_MAX_CONSECUTIVE_MISSES) {
        s_polling_enabled = false;
        onx_display_set_status("OBD POLLING PAUSED");
        ESP_LOGE(TAG,
                 "OBD polling paused after %u consecutive misses; CAN reception and touch UI remain active. "
                 "Check vehicle connection, ignition state and bitrate, then reboot to retry.",
                 (unsigned)OBD_MAX_CONSECUTIVE_MISSES);
    }
}

static void record_rx_statistics(const can_frame_t *frame)
{
    if (s_total_rx_frames == 0U) {
        onx_display_set_status("CAN TRAFFIC DETECTED");
    }
    ++s_total_rx_frames;
    ++s_rx_frames_since_diag;
    s_have_last_rx_frame = true;
    s_last_rx_extended = frame->extended;
    s_last_rx_id = frame->id;
    s_last_rx_dlc = frame->dlc;

    for (uint32_t i = 0; i < s_unique_ids_count; ++i) {
        if (s_unique_ids[i].used && s_unique_ids[i].id == frame->id &&
            s_unique_ids[i].extended == frame->extended) {
            return;
        }
    }

    if (s_unique_ids_count < OBD_RX_STATS_MAX_UNIQUE_IDS) {
        s_unique_ids[s_unique_ids_count].id = frame->id;
        s_unique_ids[s_unique_ids_count].extended = frame->extended;
        s_unique_ids[s_unique_ids_count].used = true;
        ++s_unique_ids_count;
    } else {
        s_unique_ids_table_full = true;
    }
}

static void poll_rx_overflow_flags(int64_t now_us)
{
    if (s_last_overflow_poll_us != 0 &&
        (now_us - s_last_overflow_poll_us) < OBD_RX_OVERFLOW_POLL_US) {
        return;
    }
    s_last_overflow_poll_us = now_us;

    uint8_t flags = 0;
    esp_err_t err = can_read_and_clear_rx_overflow(&flags);
    if (err != ESP_OK) {
        if (s_last_overflow_error_log_us == 0 ||
            (now_us - s_last_overflow_error_log_us) >= 1000000LL) {
            s_last_overflow_error_log_us = now_us;
            ESP_LOGW(TAG, "RX overflow poll/clear failed: %s", esp_err_to_name(err));
        }
        return;
    }

    if ((flags & OBD_RX0_OVERFLOW_FLAG) != 0) ++s_rx0_overflow_events;
    if ((flags & OBD_RX1_OVERFLOW_FLAG) != 0) ++s_rx1_overflow_events;

    if (flags != 0 && (s_last_overflow_log_us == 0 ||
        (now_us - s_last_overflow_log_us) >= 1000000LL)) {
        s_last_overflow_log_us = now_us;
        ESP_LOGW(TAG,
                 "RX overflow latched flags=0x%02X; cleared. Cumulative observations: RX0=%lu RX1=%lu (latch observations, not exact lost-frame counts)",
                 flags, (unsigned long)s_rx0_overflow_events,
                 (unsigned long)s_rx1_overflow_events);
    }
}

static void log_raw_frame(const can_frame_t *frame)
{
    char payload[3U * 8U + 1U] = {0};
    size_t used = 0;

    for (uint8_t i = 0; i < frame->dlc; ++i) {
        int written = snprintf(payload + used, sizeof(payload) - used,
                               i == 0 ? "%02X" : " %02X", frame->data[i]);
        if (written < 0 || (size_t)written >= sizeof(payload) - used) {
            break;
        }
        used += (size_t)written;
    }

    /* Show a short sample at INFO by default; keep sustained bus traffic at DEBUG. */
    if (s_raw_frames_logged < 24U) {
        ESP_LOGI(TAG, "CAN RX id=0x%0*lX %s%s dlc=%u data=[%s]",
                 frame->extended ? 8 : 3,
                 (unsigned long)frame->id,
                 frame->extended ? "EXT" : "STD",
                 frame->rtr ? " RTR" : "",
                 (unsigned)frame->dlc, payload);
    } else {
        ESP_LOGD(TAG, "CAN RX id=0x%0*lX %s%s dlc=%u data=[%s]",
                 frame->extended ? 8 : 3,
                 (unsigned long)frame->id,
                 frame->extended ? "EXT" : "STD",
                 frame->rtr ? " RTR" : "",
                 (unsigned)frame->dlc, payload);
    }
    ++s_raw_frames_logged;
}

static bool is_obd_response_id(const can_frame_t *frame)
{
    if (frame->extended) {
        return frame->id >= 0x18DAF100U && frame->id <= 0x18DAF1FFU;
    }
    return frame->id >= OBD_RESPONSE_ID_FIRST && frame->id <= OBD_RESPONSE_ID_LAST;
}

static esp_err_t send_isotp_frame(const can_frame_t *frame, void *context)
{
    (void)context;
    return can_send(frame);
}

static esp_err_t initialize_isotp_channels(void)
{
    esp_err_t err = can_get_config(&s_can_config);
    if (err != ESP_OK) return err;

    for (uint8_t index = 0; index < OBD_ECU_COUNT; ++index) {
        uint32_t tx_id;
        uint32_t rx_id;
        if (s_can_config.extended_ids) {
            rx_id = 0x18DAF110U + index;
            tx_id = 0x18DA0000U | ((rx_id & 0xFFU) << 8) | 0xF1U;
        } else {
            tx_id = 0x7E0U + index;
            rx_id = 0x7E8U + index;
        }
        const isotp_config_t config = {
            .tx_id = tx_id,
            .rx_id = rx_id,
            .extended_id = s_can_config.extended_ids,
            .padding = 0x00U,
            .rx_block_size = 0U,
            .rx_st_min = 0U,
        };
        err = isotp_init(&s_isotp[index], &config, send_isotp_frame, NULL);
        if (err != ESP_OK) return err;
        err = uds_client_init(&s_uds[index], &config, send_isotp_frame, NULL);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void log_uds_event(uint8_t ecu_index, const uds_event_t *event)
{
    if (event->type == UDS_EVENT_RESPONSE) {
        if (event->service == 0x22U) {
            ESP_LOGI(TAG,
                     "UDS ReadDataByIdentifier ECU index=%u DID=0x%04X positive response; "
                     "raw data length=%u (no OEM decoder applied)",
                     (unsigned)ecu_index, event->did, (unsigned)event->data_length);
        } else {
            ESP_LOGI(TAG, "UDS service 0x%02X ECU index=%u positive response",
                     event->service, (unsigned)ecu_index);
        }
    } else if (event->type == UDS_EVENT_NEGATIVE_RESPONSE) {
        ESP_LOGW(TAG, "UDS ECU index=%u negative response service=0x%02X DID=0x%04X NRC=0x%02X",
                 (unsigned)ecu_index, event->service, event->did, event->nrc);
    } else if (event->type == UDS_EVENT_RESPONSE_PENDING) {
        ESP_LOGI(TAG, "UDS ECU index=%u service=0x%02X response pending",
                 (unsigned)ecu_index, event->service);
    } else if (event->type == UDS_EVENT_ERROR) {
        ESP_LOGW(TAG, "UDS ECU index=%u transaction error=%u",
                 (unsigned)ecu_index, (unsigned)event->error);
    }
}

static esp_err_t validate_uds_request(uint8_t ecu_index)
{
    if (ecu_index >= OBD_ECU_COUNT) return ESP_ERR_INVALID_ARG;
#if !OBD_ACTIVE_TX_ENABLED || OBD_LOOPBACK_SIM_ENABLED
    return ESP_ERR_INVALID_STATE;
#else
    if (!s_started || !s_polling_enabled || s_waiting_for_response) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
#endif
}

esp_err_t obd_client_uds_start_session(uint8_t ecu_index)
{
    esp_err_t err = validate_uds_request(ecu_index);
    if (err != ESP_OK) return err;
    return uds_client_start_extended_session(&s_uds[ecu_index],
                                              (uint64_t)esp_timer_get_time());
}

esp_err_t obd_client_uds_tester_present(uint8_t ecu_index)
{
    esp_err_t err = validate_uds_request(ecu_index);
    if (err != ESP_OK) return err;
    return uds_client_tester_present(&s_uds[ecu_index],
                                     (uint64_t)esp_timer_get_time());
}

esp_err_t obd_client_uds_read_did(uint8_t ecu_index, uint16_t did)
{
    esp_err_t err = validate_uds_request(ecu_index);
    if (err != ESP_OK) return err;
    return uds_client_read_did(&s_uds[ecu_index], did,
                               (uint64_t)esp_timer_get_time());
}

static void record_obd_response(const can_frame_t *frame,
                                const uint8_t *payload, size_t length)
{
    if (payload == NULL || length < 2U) {
        ESP_LOGW(TAG, "Short OBD application payload from ECU 0x%0*lX",
                 frame->extended ? 8 : 3, (unsigned long)frame->id);
        return;
    }

    if (payload[0] == 0x41U) {
        const uint8_t pid = payload[1];
        const bool matching_request = s_waiting_for_response && s_requested_pid == pid;
        const int64_t now_us = esp_timer_get_time();

        if (pid == s_discovery_base) {
            if (length != 6U) {
                ESP_LOGW(TAG, "Malformed PID bitmap from ECU 0x%0*lX: length=%u",
                         frame->extended ? 8 : 3, (unsigned long)frame->id,
                         (unsigned)length);
                if (matching_request) record_miss("Malformed supported-PID bitmap");
                return;
            }
            esp_err_t err = parameter_manager_record_supported_page(
                frame->id, frame->extended, pid, &payload[2]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Could not cache ECU PID bitmap: %s", esp_err_to_name(err));
                return;
            }
            s_discovery_response_seen = true;
            s_discovery_has_continuation |=
                parameter_manager_page_has_continuation(&payload[2]);
            if (s_discovery_collect_until_us == 0) {
                s_discovery_collect_until_us = now_us + 100000LL;
            }
        } else {
            parameter_value_t value = {0};
            esp_err_t err = parameter_manager_decode_obd(
                frame->id, frame->extended, pid, &payload[2], length - 2U,
                now_us, &value);
            if (err == ESP_ERR_NOT_SUPPORTED) {
                ESP_LOGD(TAG, "No standard decoder registered for Mode 01 PID 0x%02X", pid);
                return;
            }
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "OBD decoder rejected ECU 0x%0*lX PID 0x%02X: %s",
                         frame->extended ? 8 : 3, (unsigned long)frame->id,
                         pid, esp_err_to_name(err));
                if (matching_request) record_miss("Malformed or out-of-range OBD response");
                return;
            }

            if (pid == OBD_PID_ENGINE_RPM) {
                const uint32_t rpm = (uint32_t)(value.value + 0.5f);
                onx_display_set_rpm(rpm);
                s_last_rpm_response_us = now_us;
                s_rpm_displayed_live = true;
                ++s_successful_rpm_responses;
                ESP_LOGI(TAG, "Engine RPM: %lu rpm (ECU CAN ID 0x%0*lX; response #%lu)",
                         (unsigned long)rpm, frame->extended ? 8 : 3,
                         (unsigned long)frame->id,
                         (unsigned long)s_successful_rpm_responses);
            } else if (pid == OBD_PID_COOLANT_TEMP) {
                const int32_t temp_c = (int32_t)(value.value + 0.5f);
                onx_display_set_coolant_temp(temp_c);
                s_last_coolant_response_us = now_us;
                s_coolant_displayed_live = true;
                ++s_successful_coolant_responses;
                ESP_LOGI(TAG, "Coolant temperature: %ld C (ECU CAN ID 0x%0*lX; response #%lu)",
                         (long)temp_c, frame->extended ? 8 : 3,
                         (unsigned long)frame->id,
                         (unsigned long)s_successful_coolant_responses);
            } else {
                const parameter_descriptor_t *catalog = NULL;
                size_t count = 0;
                catalog = parameter_manager_catalog(&count);
                for (size_t i = 0; i < count; ++i) {
                    if (catalog[i].pid == pid) {
                        ESP_LOGI(TAG, "%s: %.2f %s (ECU CAN ID 0x%0*lX; raw length=%u)",
                                 catalog[i].name, (double)value.value, catalog[i].unit,
                                 frame->extended ? 8 : 3,
                                 (unsigned long)frame->id, value.raw_length);
                        break;
                    }
                }
            }
            onx_display_set_status("OBD DATA LIVE");
        }

        if (matching_request) {
            s_waiting_for_response = false;
            s_requested_pid = 0U;
            s_consecutive_misses = 0U;
        }
        return;
    }

    if (payload[0] == 0x7FU) {
        if (length < 3U) {
            ESP_LOGW(TAG, "Malformed negative diagnostic response from ECU 0x%0*lX",
                     frame->extended ? 8 : 3, (unsigned long)frame->id);
            return;
        }
        ESP_LOGW(TAG, "ECU 0x%0*lX negative response service=0x%02X NRC=0x%02X",
                 frame->extended ? 8 : 3, (unsigned long)frame->id,
                 payload[1], payload[2]);
        if (s_waiting_for_response) record_miss("ECU returned a negative diagnostic response");
        return;
    }

    ESP_LOGD(TAG, "Unsupported diagnostic application payload from ECU 0x%0*lX",
             frame->extended ? 8 : 3, (unsigned long)frame->id);
}

static void process_frame(const can_frame_t *frame)
{
    record_rx_statistics(frame);
    log_raw_frame(frame);

#if OBD_LOOPBACK_SIM_ENABLED
    /*
     * Internal-only ECU emulator: CAN_MODE_LOOPBACK routes these frames back
     * through the MCP2515 RX buffers without driving the physical CAN pins.
     */
    if (!frame->extended && !frame->rtr && frame->id == OBD_REQUEST_ID_STANDARD &&
        frame->dlc >= 3U && frame->data[0] == 0x02U &&
        frame->data[1] == OBD_MODE_CURRENT_DATA) {
        can_frame_t response = {
            .id = OBD_RESPONSE_ID_FIRST,
            .dlc = 8U,
            .extended = false,
            .rtr = false,
            .data = {0U},
        };

        if (frame->data[2] == OBD_DISCOVERY_PID) {
            const uint8_t supported_response[8] = {
                0x06U, 0x41U, 0x00U, 0x08U, 0x10U, 0x00U, 0x00U, 0x00U,
            };
            memcpy(response.data, supported_response, sizeof(supported_response));
        } else if (frame->data[2] == OBD_PID_ENGINE_RPM) {
            /* 2000 rpm: raw PID value 8000 (0x1F40), scaled by 1/4. */
            const uint8_t rpm_response[8] = {0x04U, 0x41U, 0x0CU, 0x1FU,
                                              0x40U, 0x00U, 0x00U, 0x00U};
            memcpy(response.data, rpm_response, sizeof(rpm_response));
        } else if (frame->data[2] == OBD_PID_COOLANT_TEMP) {
            /* 50 C: raw PID value 90 (0x5A), offset by -40 C. */
            const uint8_t coolant_response[8] = {0x03U, 0x41U, 0x05U, 0x5AU,
                                                  0x00U, 0x00U, 0x00U, 0x00U};
            memcpy(response.data, coolant_response, sizeof(coolant_response));
        } else {
            /* Mode 01 PID not supported: negative response NRC 0x11. */
            const uint8_t negative_response[8] = {0x03U, 0x7FU, 0x01U, 0x11U,
                                                   0x00U, 0x00U, 0x00U, 0x00U};
            memcpy(response.data, negative_response, sizeof(negative_response));
        }

        esp_err_t tx_err = can_send(&response);
        if (tx_err == ESP_OK) {
            ESP_LOGI(TAG, "Loopback ECU simulator replied to PID 0x%02X on ID 0x%03lX",
                     frame->data[2], (unsigned long)response.id);
        } else {
            ESP_LOGE(TAG, "Loopback ECU simulator TX failed: %s", esp_err_to_name(tx_err));
        }
        return;
    }
#endif

    if (frame->rtr || !is_obd_response_id(frame)) return;

    for (uint8_t i = 0; i < OBD_ECU_COUNT; ++i) {
        const isotp_event_type_t result = isotp_on_frame(
            &s_isotp[i], frame, (uint64_t)esp_timer_get_time(), &s_isotp_event);
        if (result == ISOTP_EVENT_PDU_RECEIVED) {
            record_obd_response(frame, s_isotp_event.data, s_isotp_event.length);
        } else if (result == ISOTP_EVENT_ERROR) {
            ESP_LOGW(TAG, "ISO-TP RX error ECU=0x%0*lX reason=%u",
                     frame->extended ? 8 : 3, (unsigned long)frame->id,
                     (unsigned)s_isotp_event.error);
        }

        const uds_event_type_t uds_result = uds_client_on_frame(
            &s_uds[i], frame, (uint64_t)esp_timer_get_time(), &s_uds_event);
        if (uds_result != UDS_EVENT_NONE) log_uds_event(i, &s_uds_event);
    }
}

static void log_controller_diagnostics(int64_t now_us)
{
    if (s_last_diag_attempt_us != 0 &&
        (now_us - s_last_diag_attempt_us) < 5000000LL) {
        return;
    }
    s_last_diag_attempt_us = now_us;

    can_diagnostics_t diag;
    esp_err_t err = can_read_diagnostics(&diag);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not read MCP2515 diagnostics: %s", esp_err_to_name(err));
        return;
    }

    const bool first_diag = (s_last_diag_success_us == 0);
    uint32_t interval_frames = first_diag ? 0U : s_rx_frames_since_diag;
    int64_t interval_us = first_diag ? 0 : now_us - s_last_diag_success_us;
    /* Start every successful diagnostic interval from a clean RX-frame baseline. */
    s_rx_frames_since_diag = 0;
    s_last_diag_success_us = now_us;

    const uint64_t avg_fps_x100 = interval_us > 0
        ? ((uint64_t)interval_frames * 100000000ULL) / (uint64_t)interval_us
        : 0;
    const bool listen_only = (diag.canstat & 0xE0U) == 0x60U;
    const char *counter_note = listen_only
        ? "; TEC/REC inactive in Listen-Only; not bitrate evidence"
        : "";

    if (s_have_last_rx_frame) {
        ESP_LOGI(TAG,
                 "MCP2515 diag: CANSTAT=0x%02X EFLG=0x%02X TEC=%u REC=%u TXREQ-mask=0x%X RX-overflow=%s; RXavg=%lu.%02lu fps total=%lu unique-IDs=%lu%s last=%s0x%0*lX/DLC=%u RXOVR0=%lu RXOVR1=%lu%s",
                 diag.canstat, diag.eflg, diag.tec, diag.rec, diag.txreq_mask,
                 (diag.eflg & 0xC0U) ? "YES" : "no",
                 (unsigned long)(avg_fps_x100 / 100ULL),
                 (unsigned long)(avg_fps_x100 % 100ULL),
                 (unsigned long)s_total_rx_frames,
                 (unsigned long)s_unique_ids_count,
                 s_unique_ids_table_full ? "+ (table full)" : "",
                 s_last_rx_extended ? "EXT:" : "STD:",
                 s_last_rx_extended ? 8 : 3,
                 (unsigned long)s_last_rx_id, (unsigned)s_last_rx_dlc,
                 (unsigned long)s_rx0_overflow_events,
                 (unsigned long)s_rx1_overflow_events, counter_note);
    } else {
        ESP_LOGI(TAG,
                 "MCP2515 diag: CANSTAT=0x%02X EFLG=0x%02X TEC=%u REC=%u TXREQ-mask=0x%X RX-overflow=%s; RXavg=0.00 fps total=0 unique-IDs=0 last=none RXOVR0=%lu RXOVR1=%lu%s",
                 diag.canstat, diag.eflg, diag.tec, diag.rec, diag.txreq_mask,
                 (diag.eflg & 0xC0U) ? "YES" : "no",
                 (unsigned long)s_rx0_overflow_events,
                 (unsigned long)s_rx1_overflow_events, counter_note);
    }
}

static uint8_t select_next_pid(int64_t now_us)
{
    if (!s_discovery_complete) return s_discovery_base;

    size_t count = 0;
    const parameter_descriptor_t *catalog = parameter_manager_catalog(&count);
    const parameter_descriptor_t *selected = NULL;
    for (size_t i = 0; i < count; ++i) {
        const parameter_descriptor_t *candidate = &catalog[i];
        if (!parameter_manager_any_ecu_supports(candidate->pid)) continue;
        if (s_next_due_us[candidate->pid] != 0 &&
            now_us < s_next_due_us[candidate->pid]) continue;
        if (selected == NULL || candidate->priority < selected->priority ||
            (candidate->priority == selected->priority &&
             s_next_due_us[candidate->pid] < s_next_due_us[selected->pid])) {
            selected = candidate;
        }
    }
    return selected != NULL ? selected->pid : 0xFFU;
}

esp_err_t obd_client_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

#if OBD_LOOPBACK_SIM_ENABLED
    esp_err_t err = can_set_mode(CAN_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter CAN Configuration mode for loopback simulator: %s", esp_err_to_name(err));
        return err;
    }
    err = can_set_mode(CAN_MODE_LOOPBACK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter MCP2515 Loopback mode for ECU simulator: %s", esp_err_to_name(err));
        return err;
    }
    s_polling_enabled = true;
    s_next_request_us = esp_timer_get_time();
    ESP_LOGW(TAG, "INTERNAL OBD-II ECU SIMULATOR ENABLED: MCP2515 loopback only; no physical CAN frames are transmitted");
    onx_display_show_waiting_for_rpm("INTERNAL OBD SIM");
#elif OBD_ACTIVE_TX_ENABLED
    esp_err_t err = can_set_mode(CAN_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter CAN Configuration mode for RX filters: %s", esp_err_to_name(err));
        return err;
    }
    err = can_configure_obd_response_filters();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot configure OBD response filters: %s", esp_err_to_name(err));
        return err;
    }
    err = can_set_mode(CAN_MODE_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter CAN Normal mode: %s", esp_err_to_name(err));
        return err;
    }
    s_polling_enabled = true;
    s_next_request_us = esp_timer_get_time();
    ESP_LOGW(TAG, "ACTIVE OBD TX enabled; scheduler will discover supported PID ranges before polling");
    onx_display_show_waiting_for_rpm("ACTIVE OBD REQUESTS");
#else
    esp_err_t err = can_set_mode(CAN_MODE_LISTEN_ONLY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter CAN Listen-Only mode: %s", esp_err_to_name(err));
        return err;
    }
    s_polling_enabled = false;
    ESP_LOGI(TAG, "Passive Listen-Only monitor selected; no CAN frames will be transmitted");
    onx_display_show_waiting_for_rpm("LISTEN ONLY - WAITING");
#endif

    parameter_manager_init();
    err = initialize_isotp_channels();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialize ISO-TP endpoints: %s", esp_err_to_name(err));
        return err;
    }

    s_started = true;
    s_waiting_for_response = false;
    s_requested_pid = 0U;
    s_discovery_base = OBD_DISCOVERY_PID;
    s_discovery_complete = false;
    s_discovery_response_seen = false;
    s_discovery_has_continuation = false;
    s_discovery_collect_until_us = 0;
    s_consecutive_misses = 0;
    s_successful_rpm_responses = 0;
    s_successful_coolant_responses = 0;
    s_last_rpm_response_us = 0;
    s_rpm_displayed_live = false;
    s_last_coolant_response_us = 0;
    s_coolant_displayed_live = false;
    s_raw_frames_logged = 0;
    s_last_diag_attempt_us = 0;
    s_last_diag_success_us = 0;
    s_last_rx_error_log_us = 0;
    s_rx_error_count = 0;
    s_last_overflow_poll_us = 0;
    s_last_overflow_log_us = 0;
    s_last_overflow_error_log_us = 0;
    s_rx0_overflow_events = 0;
    s_rx1_overflow_events = 0;
    s_total_rx_frames = 0;
    s_rx_frames_since_diag = 0;
    s_have_last_rx_frame = false;
    s_last_rx_extended = false;
    s_last_rx_id = 0;
    s_last_rx_dlc = 0;
    s_unique_ids_count = 0;
    s_unique_ids_table_full = false;
    memset(s_unique_ids, 0, sizeof(s_unique_ids));
    memset(s_next_due_us, 0, sizeof(s_next_due_us));
    s_last_scheduler_lag_us = 0;
    ESP_LOGI(TAG, "Monitor ready: configured bitrate=%lu kbps CAN IDs=%s; physical bus not yet verified",
             (unsigned long)(s_can_config.bitrate / 1000U),
             s_can_config.extended_ids ? "29-bit" : "11-bit");
#if OBD_ACTIVE_TX_ENABLED && !OBD_LOOPBACK_SIM_ENABLED
    ESP_LOGI(TAG, "OBD response filters configured for %s addressing",
             s_can_config.extended_ids ? "29-bit" : "11-bit");
#endif
#if OBD_LOOPBACK_SIM_ENABLED
    ESP_LOGI(TAG, "Loopback simulator accepts internal test frames only; physical CAN bus remains isolated by MCP2515 Loopback mode");
#endif
    ESP_LOGI(TAG, "First 24 raw CAN frames log at INFO, later frames at DEBUG; OBD PIDs use the Parameter Manager and ISO-TP");
    return ESP_OK;
}

void obd_client_poll(void)
{
    if (!s_started) {
        return;
    }

    /* Drain both MCP2515 RX buffers regularly; cap work per pass to stay responsive. */
    for (int i = 0; i < OBD_MAX_RX_FRAMES_PER_STEP; ++i) {
        can_frame_t frame = {0};
        esp_err_t err = can_receive(&frame);
        if (err == ESP_ERR_NOT_FOUND) {
            break;
        }
        if (err != ESP_OK) {
            ++s_rx_error_count;
            const int64_t error_now_us = esp_timer_get_time();
            if (s_last_rx_error_log_us == 0 ||
                (error_now_us - s_last_rx_error_log_us) >= 1000000LL) {
                s_last_rx_error_log_us = error_now_us;
                ESP_LOGW(TAG, "CAN receive error %s (occurrences=%lu)",
                         esp_err_to_name(err), (unsigned long)s_rx_error_count);
            }
            break;
        }
        process_frame(&frame);
    }

    const int64_t now_us = esp_timer_get_time();
    for (uint8_t i = 0; i < OBD_ECU_COUNT; ++i) {
        const isotp_event_type_t result = isotp_tick(&s_isotp[i],
            (uint64_t)now_us, &s_isotp_event);
        if (result == ISOTP_EVENT_ERROR) {
            ESP_LOGW(TAG, "ISO-TP timeout/error endpoint=%u reason=%u",
                     (unsigned)i, (unsigned)s_isotp_event.error);
        }

        const uds_event_type_t uds_result = uds_client_tick(
            &s_uds[i], (uint64_t)now_us, &s_uds_event);
        if (uds_result != UDS_EVENT_NONE) log_uds_event(i, &s_uds_event);
    }
    parameter_manager_mark_stale(now_us);

    /* Each live page follows its own requested update interval. */
    if (s_rpm_displayed_live && s_last_rpm_response_us != 0 &&
        (now_us - s_last_rpm_response_us) >= 500000LL) {
        s_rpm_displayed_live = false;
        s_last_rpm_response_us = 0;
        onx_display_clear_rpm();
        ESP_LOGW(TAG, "RPM data stale; RPM page value cleared until a fresh response arrives");
    }
    if (s_coolant_displayed_live && s_last_coolant_response_us != 0 &&
        (now_us - s_last_coolant_response_us) >= 3500000LL) {
        s_coolant_displayed_live = false;
        s_last_coolant_response_us = 0;
        onx_display_clear_coolant_temp();
        ESP_LOGW(TAG, "Coolant-temperature data stale; value cleared until a fresh response arrives");
    }
    poll_rx_overflow_flags(now_us);
    log_controller_diagnostics(now_us);
    if (s_waiting_for_response && (now_us - s_request_sent_us) >= OBD_RESPONSE_TIMEOUT_US) {
        parameter_manager_record_timeout(s_requested_pid, now_us);
        ESP_LOGW(TAG, "OBD Mode 01 PID 0x%02X response timeout after 200 ms",
                 s_requested_pid);
        record_miss("No response for outstanding OBD request");
    }

#if !OBD_SCHEDULER_ENABLED
    return;
#else
    bool uds_transaction_active = false;
    for (uint8_t i = 0; i < OBD_ECU_COUNT; ++i) {
        uds_transaction_active |= uds_client_transaction_active(&s_uds[i]);
    }
    if (!s_polling_enabled || s_waiting_for_response || uds_transaction_active ||
        now_us < s_next_request_us) return;

    if (!s_discovery_complete && s_discovery_collect_until_us != 0 &&
        now_us >= s_discovery_collect_until_us) {
        if (s_discovery_response_seen && s_discovery_has_continuation &&
            s_discovery_base < OBD_MAX_DISCOVERY_BASE) {
            s_discovery_base = (uint8_t)(s_discovery_base + OBD_PID_PAGE_SIZE);
            ESP_LOGI(TAG, "Supported-PID discovery advancing to bitmap 0x%02X",
                     s_discovery_base);
        } else {
            s_discovery_complete = true;
            ESP_LOGI(TAG, "Supported-PID discovery complete at bitmap 0x%02X",
                     s_discovery_base);
        }
        s_discovery_response_seen = false;
        s_discovery_has_continuation = false;
        s_discovery_collect_until_us = 0;
        s_next_request_us = now_us;
        if (s_discovery_complete) return;
    }

    if ((!s_discovery_complete && s_discovery_collect_until_us != 0) ||
        !s_polling_enabled) {
        return;
    }

    const uint8_t request_pid = select_next_pid(now_us);
    if (request_pid == 0xFFU) return;
    const uint32_t request_id = s_can_config.extended_ids
        ? OBD_REQUEST_ID_EXTENDED : OBD_REQUEST_ID_STANDARD;
    const can_frame_t request = {
        .id = request_id,
        .dlc = 8,
        .extended = s_can_config.extended_ids,
        .data = {0x02U, OBD_MODE_CURRENT_DATA, request_pid, 0x00U,
                 0x00U, 0x00U, 0x00U, 0x00U},
    };

    esp_err_t err = can_send(&request);
    s_next_request_us = now_us + OBD_REQUEST_PERIOD_US;
    if (err == ESP_OK) {
        s_waiting_for_response = true;
        s_requested_pid = request_pid;
        s_request_sent_us = now_us;
        if (s_discovery_complete) {
            size_t count = 0;
            const parameter_descriptor_t *catalog = parameter_manager_catalog(&count);
            for (size_t i = 0; i < count; ++i) {
                if (catalog[i].pid == request_pid) {
                    s_next_due_us[request_pid] = now_us +
                        (int64_t)catalog[i].update_period_ms * 1000LL;
                    break;
                }
            }
        }
        ESP_LOGI(TAG, "OBD Mode 01 request transmitted: ID=0x%0*lX PID=0x%02X%s",
                 s_can_config.extended_ids ? 8 : 3, (unsigned long)request_id,
                 request_pid, request_pid <= OBD_MAX_DISCOVERY_BASE &&
                 (request_pid % OBD_PID_PAGE_SIZE) == 0U ? " (SUPPORTED PID BITMAP)" : "");
    } else {
        record_miss("OBD request transmission failed");
        can_diagnostics_t diag;
        esp_err_t diag_err = can_read_diagnostics(&diag);
        if (diag_err == ESP_OK) {
            ESP_LOGW(TAG,
                     "TX diagnostic: result=%s CANSTAT=0x%02X EFLG=0x%02X TEC=%u REC=%u TXREQ-mask=0x%X",
                     esp_err_to_name(err), diag.canstat, diag.eflg,
                     diag.tec, diag.rec, diag.txreq_mask);
        } else {
            ESP_LOGW(TAG, "TX attempt result=%s; diagnostic read also failed: %s",
                     esp_err_to_name(err), esp_err_to_name(diag_err));
        }
    }
#endif
}
