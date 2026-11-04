#include "obd_client.h"

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "can_driver.h"
#include "can_discovery.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs.h"
#if __has_include("esp_littlefs.h")
#include "esp_littlefs.h"
#endif
#include "isotp.h"
#include "onx_display.h"
#include "parameter_manager.h"
#include "pid_scheduler.h"
#include "sdkconfig.h"
#include "uds_client.h"
#include "oem_did_scan.h"
#include "did_profile.h"
#if defined(CONFIG_OBD_RUN_PROTOCOL_TESTS) && CONFIG_OBD_RUN_PROTOCOL_TESTS
#include "obd_client_test.h"
#endif

static const char *TAG = "obd_client";

static void format_hex_bytes(const uint8_t *data, size_t length, char *buffer,
                             size_t buffer_len);
static void obd_client_did_scan_handle_event(uint8_t ecu_index,
                                             const uds_event_t *event);
static void did_poll_handle_event(uint8_t ecu_index, const uds_event_t *event);
static void did_poll_arm_after_identity(void);
static void did_poll_reset(void);
static void safe_copy_string(char *dest, size_t dest_size, const char *src)
{
    if (dest == NULL || dest_size == 0U) {
        return;
    }
    dest[0] = '\0';
    if (src == NULL) {
        return;
    }
    const size_t max_len = dest_size - 1U;
    if (max_len == 0U) {
        return;
    }
    const size_t copy_len = (strnlen(src, max_len + 1U) > max_len) ? max_len : strnlen(src, max_len + 1U);
    memcpy(dest, src, copy_len);
    dest[copy_len] = '\0';
}

static obd_pid_status_t s_pid_status[256];

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
#define CAN_DISCOVERY_LISTEN_US     300000LL
#define CAN_DISCOVERY_RETRY_US     5000000LL
#define CAN_DISCOVERY_VERIFY_ATTEMPTS 2U
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
static bool s_pid_polling_enabled = true;
static bool s_waiting_for_response;
static bool s_vehicle_identification_started;
static bool s_vehicle_identification_pending;
static size_t s_vehicle_identity_queue_index;
static bool s_auto_identity_attempted;
static uint8_t s_mode09_support_bytes[8];
static uint8_t s_requested_pid;
#if OBD_SCHEDULER_ENABLED
static int64_t s_next_request_us;
#endif
static int64_t s_request_sent_us;
static uint32_t s_consecutive_misses;
static uint32_t s_successful_rpm_responses;
static uint32_t s_successful_coolant_responses;
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
static obd_vehicle_identity_t s_vehicle_identity;
static can_bus_config_t s_can_config;
static can_discovery_t s_can_discovery;
static uint8_t s_verify_timeouts;
static bool s_can_discovery_enabled;
static int64_t s_candidate_deadline_us;
static int64_t s_discovery_retry_us;
static uds_client_t s_uds[OBD_ECU_COUNT];
static uds_event_t s_uds_event;
static uint32_t s_uds_rx_ids[OBD_ECU_COUNT];
static bool s_uds_channel_used[OBD_ECU_COUNT];
static bool s_pid_source_valid[256];
static uint32_t s_pid_source_id[256];
static bool s_pid_source_extended[256];
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

typedef enum {
    OBD_DID_SCAN_STATE_IDLE = 0,
    OBD_DID_SCAN_STATE_RUNNING,
    OBD_DID_SCAN_STATE_COMPLETED,
    OBD_DID_SCAN_STATE_ERROR,
} obd_did_scan_status_t;

typedef struct {
    bool active;
    obd_did_scan_status_t state;
    uint8_t ecu_index;
    diag_ecu_role_t role;
    const uds_did_descriptor_t *queue[512];
    size_t queue_count;
    size_t queue_index;
    uint16_t current_did;
    bool waiting_for_response;
    int64_t request_sent_us;
    int64_t deadline_us;
    uint32_t positive_count;
    uint32_t negative_count;
    uint32_t timeout_count;
    uint32_t total_count;
    bool allow_experimental;
    bool fs_ready;
    FILE *session_file;
    char session_path[128];
    int64_t session_started_us;
} obd_did_scan_state_t;

static obd_did_scan_state_t s_did_scan;
/* Kept outside s_did_scan because each ECU phase resets the scan state. */
static uint32_t s_did_scan_session_counter;
static bool s_did_scan_powertrain_active;
static bool s_did_scan_tcu_pending;

typedef enum {
    DID_POLL_PHASE_IDLE = 0,
    DID_POLL_PHASE_DISCOVERY, /* round 1: ask every candidate DID once */
    DID_POLL_PHASE_ACTIVE,    /* only DIDs the ECU did not reject with NRC 0x31 */
} did_poll_phase_t;

static struct {
    did_poll_phase_t phase;
    did_poll_set_t set;
    did_brand_t brand;
    int current;
    bool waiting;
    bool last_was_did;
    int64_t sent_us;
    uint32_t positive_count;
} s_dp;

/* Identity DIDs queried by the dedicated V command; D skips these duplicates. */
static const uint16_t s_vehicle_identity_dids[] = {
    0xF190U, 0xF187U, 0xF189U, 0xF191U, 0xF1A3U,
    0xF197U, 0xF1ADU, 0xF19EU, 0xF1A2U,
    0xF1AAU, 0xF17CU, 0xF1F4U, 0xF1A5U, 0xF804U, 0xF806U,
};

static const char *obd_did_scan_state_name(obd_did_scan_status_t state)
{
    switch (state) {
    case OBD_DID_SCAN_STATE_IDLE:
        return "IDLE";
    case OBD_DID_SCAN_STATE_RUNNING:
        return "RUNNING";
    case OBD_DID_SCAN_STATE_COMPLETED:
        return "COMPLETED";
    case OBD_DID_SCAN_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

static void obd_client_did_scan_close_session(void)
{
    if (s_did_scan.session_file != NULL) {
        fflush(s_did_scan.session_file);
        fclose(s_did_scan.session_file);
        s_did_scan.session_file = NULL;
    }
    if (s_did_scan.session_path[0] != '\0') {
        ESP_LOGI(TAG, "[OEM_DID_SCAN] session written to %s", s_did_scan.session_path);
    }
    s_did_scan.session_path[0] = '\0';
}

static bool obd_client_did_scan_prepare_session_file(const char *profile_id)
{
#if __has_include("esp_littlefs.h")
    static bool littlefs_registered = false;
    if (s_did_scan.fs_ready && s_did_scan.session_file != NULL) {
        return true;
    }
    if (!littlefs_registered) {
        const esp_vfs_littlefs_conf_t conf = {
            .base_path = "/littlefs",
            .partition_label = "littlefs",
            .format_if_mount_failed = false,
            .dont_mount = false,
        };
        esp_err_t err = esp_vfs_littlefs_register(&conf);
        if (err != ESP_OK && err != ESP_ERR_NOT_FOUND && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "[OEM_DID_SCAN] LittleFS registration failed: %s; logging to UART only",
                    esp_err_to_name(err));
            return false;
        }
        littlefs_registered = true;
    }
    if (mkdir("/littlefs", 0777) != 0 && errno != EEXIST) {
        ESP_LOGW(TAG, "[OEM_DID_SCAN] Could not create /littlefs: %s", strerror(errno));
        return false;
    }
    if (mkdir("/littlefs/oem_did_scan", 0777) != 0 && errno != EEXIST) {
        ESP_LOGW(TAG, "[OEM_DID_SCAN] Could not create /littlefs/oem_did_scan: %s", strerror(errno));
        return false;
    }

    const int64_t now_us = esp_timer_get_time();
    const uint32_t now_s = (uint32_t)(now_us / 1000000ULL);
    s_did_scan_session_counter += 1U;
    snprintf(s_did_scan.session_path, sizeof(s_did_scan.session_path),
             "/littlefs/oem_did_scan/session_%lu_%lu.txt",
             (unsigned long)now_s,
             (unsigned long)s_did_scan_session_counter);
    s_did_scan.session_file = fopen(s_did_scan.session_path, "w");
    if (s_did_scan.session_file == NULL) {
        s_did_scan.session_path[0] = '\0';
        return false;
    }
    s_did_scan.fs_ready = true;
    fprintf(s_did_scan.session_file,
            "[OEM_DID_SCAN_SESSION]\n"
            "PROFILE=%s\n"
            "STATE=%s\n"
            "STARTED_US=%lld\n",
            profile_id != NULL ? profile_id : "unknown",
            obd_did_scan_state_name(s_did_scan.state),
            (long long)now_us);
    fflush(s_did_scan.session_file);
    return true;
#else
    (void)profile_id;
    ESP_LOGW(TAG,
             "[OEM_DID_SCAN] LittleFS support is unavailable in this build; file logging disabled and UART logging remains active");
    return false;
#endif
}

static void obd_client_did_scan_append_record(const char *scope,
                                             uint16_t did,
                                             const uint8_t *tx_data,
                                             size_t tx_length,
                                             const uint8_t *rx_data,
                                             size_t rx_length,
                                             const char *status,
                                             uint8_t nrc,
                                             bool multiframe,
                                             int64_t duration_ms)
{
    if (s_did_scan.session_file == NULL) {
        return;
    }
    char tx_hex[64] = {0};
    char rx_hex[256] = {0};
    format_hex_bytes(tx_data, tx_length, tx_hex, sizeof(tx_hex));
    format_hex_bytes(rx_data, rx_length, rx_hex, sizeof(rx_hex));

    fprintf(s_did_scan.session_file,
            "[OEM_DID_SCAN]\n"
            "TIME=%lld\n"
            "ECU=%s\n"
            "CAN_FORMAT=%s\n"
            "BITRATE=%lu\n"
            "TX_CAN_ID=0x%0*X\n"
            "RX_CAN_ID=0x%0*X\n"
            "DID=0x%04X\n"
            "TX_UDS=[%s]\n"
            "RX_UDS=[%s]\n"
            "STATUS=%s\n"
            "NRC=0x%02X\n"
            "MULTI_FRAME=%s\n"
            "DATA_LENGTH=%zu\n"
            "DURATION_MS=%lld\n",
            (long long)(esp_timer_get_time() / 1000LL),
            oem_did_scan_role_name(s_did_scan.role),
            s_can_config.extended_ids ? "29BIT" : "11BIT",
            (unsigned long)(s_can_config.bitrate / 1000U),
            s_can_config.extended_ids ? 8 : 3,
            (unsigned) (s_uds[s_did_scan.ecu_index].transport.config.tx_id),
            s_can_config.extended_ids ? 8 : 3,
            (unsigned)(s_uds[s_did_scan.ecu_index].transport.config.rx_id),
            did,
            tx_hex,
            rx_hex,
            status,
            nrc,
            multiframe ? "YES" : "NO",
            rx_length,
            (long long)duration_ms);
    fflush(s_did_scan.session_file);
    (void)scope;
}

static void record_miss(const char *reason)
{
    s_waiting_for_response = false;
    s_requested_pid = 0U;
    onx_display_set_status("OBD REQUEST FAILED");
    ++s_consecutive_misses;
    ESP_LOGW(TAG, "%s (%lu/%u)", reason,
             (unsigned long)s_consecutive_misses,
             (unsigned)OBD_MAX_CONSECUTIVE_MISSES);

    if (s_consecutive_misses >= OBD_MAX_CONSECUTIVE_MISSES)
        ESP_LOGW(TAG, "OBD polling continues despite consecutive misses");
}

static void record_pid_error(uint8_t pid, obd_pid_error_t error)
{
    ++s_pid_status[pid].error_count;
    s_pid_status[pid].last_error = error;
}

bool obd_client_get_pid_status(uint8_t pid, obd_pid_status_t *status)
{
    if (status == NULL) return false;
    *status = s_pid_status[pid];
    return status->last_request_us != 0 || status->last_success_us != 0 ||
           status->error_count != 0;
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

static void refresh_dashboard_catalog(void)
{
    static parameter_descriptor_t supported[128];
    size_t catalog_count = 0U;
    size_t supported_count = 0U;
    const parameter_descriptor_t *catalog = parameter_manager_catalog(&catalog_count);
    for (size_t i = 0; i < catalog_count; ++i) {
        if (supported_count >= sizeof(supported) / sizeof(supported[0])) {
            ESP_LOGW(TAG, "Dashboard catalog truncated at %u PIDs",
                     (unsigned)supported_count);
            break;
        }
        if (parameter_manager_any_ecu_supports(catalog[i].pid)) {
            supported[supported_count++] = catalog[i];
        }
    }
    onx_display_set_catalog(supported, supported_count);
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

static void obd_vehicle_reset_identity_state(void)
{
    memset(&s_vehicle_identity, 0, sizeof(s_vehicle_identity));
    memset(s_mode09_support_bytes, 0, sizeof(s_mode09_support_bytes));
    s_vehicle_identity.match_level = OBD_VEHICLE_MATCH_UNKNOWN;
    snprintf(s_vehicle_identity.model_year, sizeof(s_vehicle_identity.model_year), "UNKNOWN");
    snprintf(s_vehicle_identity.ecu_profile, sizeof(s_vehicle_identity.ecu_profile), "UNKNOWN");
    snprintf(s_vehicle_identity.vin, sizeof(s_vehicle_identity.vin), "UNKNOWN");
    snprintf(s_vehicle_identity.calibration_id, sizeof(s_vehicle_identity.calibration_id), "UNKNOWN");
    snprintf(s_vehicle_identity.cvn, sizeof(s_vehicle_identity.cvn), "UNKNOWN");
    snprintf(s_vehicle_identity.f187, sizeof(s_vehicle_identity.f187), "UNKNOWN");
    snprintf(s_vehicle_identity.f189, sizeof(s_vehicle_identity.f189), "UNKNOWN");
    snprintf(s_vehicle_identity.f191, sizeof(s_vehicle_identity.f191), "UNKNOWN");
    snprintf(s_vehicle_identity.f1a3, sizeof(s_vehicle_identity.f1a3), "UNKNOWN");
    snprintf(s_vehicle_identity.f197, sizeof(s_vehicle_identity.f197), "UNKNOWN");
    snprintf(s_vehicle_identity.f1ad, sizeof(s_vehicle_identity.f1ad), "UNKNOWN");
    snprintf(s_vehicle_identity.f19e, sizeof(s_vehicle_identity.f19e), "UNKNOWN");
    snprintf(s_vehicle_identity.f1a2, sizeof(s_vehicle_identity.f1a2), "UNKNOWN");
    snprintf(s_vehicle_identity.f1aa, sizeof(s_vehicle_identity.f1aa), "UNKNOWN");
    snprintf(s_vehicle_identity.f17c, sizeof(s_vehicle_identity.f17c), "UNKNOWN");
    snprintf(s_vehicle_identity.f1f4, sizeof(s_vehicle_identity.f1f4), "UNKNOWN");
    snprintf(s_vehicle_identity.f1a5, sizeof(s_vehicle_identity.f1a5), "UNKNOWN");
}

static bool obd_vehicle_vin_is_valid(const char *vin)
{
    if (vin == NULL || vin[0] == '\0') {
        return false;
    }
    size_t len = strlen(vin);
    if (len != 17U) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        const unsigned char c = (unsigned char)vin[i];
        if (!isalnum(c)) {
            return false;
        }
        if (c == 'I' || c == 'O' || c == 'Q' || c == 'i' || c == 'o' || c == 'q') {
            return false;
        }
    }
    return true;
}

static bool obd_vehicle_mode09_supports_pid(uint8_t pid)
{
    if (!s_vehicle_identity.mode09_support_known || pid > 0x3FU) {
        return false;
    }
    const uint8_t byte_index = (uint8_t)(pid / 8U);
    const uint8_t bit_index = (uint8_t)(pid % 8U);
    if (byte_index >= sizeof(s_mode09_support_bytes)) {
        return false;
    }
    return (s_mode09_support_bytes[byte_index] & (uint8_t)(1U << (7U - bit_index))) != 0U;
}

static void send_mode09_request(uint8_t pid);

static void obd_vehicle_apply_mode09_bitmap(const uint8_t *payload, size_t length)
{
    if (payload == NULL || length == 0U) {
        return;
    }

    memset(s_mode09_support_bytes, 0, sizeof(s_mode09_support_bytes));
    memset(s_vehicle_identity.mode09_support_bitmap, 0, sizeof(s_vehicle_identity.mode09_support_bitmap));
    const size_t bytes = (length < sizeof(s_mode09_support_bytes)) ? length : sizeof(s_mode09_support_bytes);
    static const char hex_digits[] = "0123456789ABCDEF";
    for (size_t i = 0; i < bytes; ++i) {
        s_mode09_support_bytes[i] = payload[i];
        s_vehicle_identity.mode09_support_bitmap[i * 2U] = hex_digits[(payload[i] >> 4U) & 0x0FU];
        s_vehicle_identity.mode09_support_bitmap[(i * 2U) + 1U] = hex_digits[payload[i] & 0x0FU];
    }
    s_vehicle_identity.mode09_support_bitmap[bytes * 2U] = '\0';
    s_vehicle_identity.mode09_support_known = true;
    ESP_LOGI(TAG, "Mode 09 support bitmap=%s", s_vehicle_identity.mode09_support_bitmap);
}

static void obd_vehicle_apply_vag_year_from_vin(const char *vin)
{
    if (vin == NULL || !obd_vehicle_vin_is_valid(vin)) {
        snprintf(s_vehicle_identity.model_year, sizeof(s_vehicle_identity.model_year), "UNKNOWN");
        s_vehicle_identity.year_known = false;
        return;
    }

    static const char *year_chars = "ABCDEFGHJKLMNPRSTVWXY123456789";
    static const int year_cycle_base[] = {1980, 1980 + 30, 1980 + 30 * 2, 1980 + 30 * 3};
    const char year_char = vin[9];
    const char *year_pos = strchr(year_chars, year_char);
    if (year_pos == NULL) {
        snprintf(s_vehicle_identity.model_year, sizeof(s_vehicle_identity.model_year), "UNKNOWN");
        s_vehicle_identity.year_known = false;
        return;
    }

    const int year_index = (int)(year_pos - year_chars);
    int year = 0;
    if (year_index < 30) {
        year = year_cycle_base[0] + year_index;
    } else if (year_index < 60) {
        year = year_cycle_base[1] + (year_index - 30);
    } else if (year_index < 90) {
        year = year_cycle_base[2] + (year_index - 60);
    } else {
        year = year_cycle_base[3] + (year_index - 90);
    }
    if (year < 1980) {
        snprintf(s_vehicle_identity.model_year, sizeof(s_vehicle_identity.model_year), "UNKNOWN");
        s_vehicle_identity.year_known = false;
        return;
    }
    char year_buf[16] = {0};
    snprintf(year_buf, sizeof(year_buf), "%d", year);
    safe_copy_string(s_vehicle_identity.model_year, sizeof(s_vehicle_identity.model_year), year_buf);
    s_vehicle_identity.year_known = true;
}

static bool obd_vehicle_string_known(const char *value)
{
    return value != NULL && value[0] != '\0' && strcmp(value, "UNKNOWN") != 0;
}

static bool obd_vehicle_vin_is_bmw(const char *vin)
{
    if (vin == NULL || strlen(vin) < 3U) return false;
    return strncmp(vin, "WBA", 3U) == 0 || strncmp(vin, "WBS", 3U) == 0;
}

static bool obd_vehicle_vin_is_audi(const char *vin)
{
    if (vin == NULL || strlen(vin) < 3U) return false;
    return strncmp(vin, "WAU", 3U) == 0 ||
           strncmp(vin, "WUA", 3U) == 0 ||
           strncmp(vin, "TRU", 3U) == 0;
}

static bool obd_vehicle_vin_is_skoda(const char *vin)
{
    if (vin == NULL || strlen(vin) < 3U) return false;
    return strncmp(vin, "TMB", 3U) == 0;
}

static bool obd_vehicle_contains_token(const char *value, const char *token)
{
    if (value == NULL || token == NULL) return false;
    return strstr(value, token) != NULL;
}

static void obd_vehicle_copy_ascii(char *dest, size_t dest_size,
                                   const uint8_t *data, size_t length)
{
    if (dest == NULL || dest_size == 0U) return;
    dest[0] = '\0';
    if (data == NULL || length == 0U) return;
    size_t out = 0U;
    for (size_t i = 0U; i < length && out + 1U < dest_size; ++i) {
        const uint8_t ch = data[i];
        if (ch == 0U || ch == 0xFFU) break;
        if (ch < 0x20U || ch > 0x7EU) {
            dest[0] = '\0';
            return;
        }
        dest[out++] = (char)ch;
    }
    dest[out] = '\0';
}

static void obd_vehicle_apply_known_profile_from_fingerprint(void)
{
    bool profile_recognized = false;
    if (obd_vehicle_vin_is_bmw(s_vehicle_identity.vin) &&
        obd_vehicle_string_known(s_vehicle_identity.f197) &&
        (obd_vehicle_contains_token(s_vehicle_identity.f197, "N47") ||
         obd_vehicle_contains_token(s_vehicle_identity.f197, "DDE7"))) {
        safe_copy_string(s_vehicle_identity.ecu_profile,
                         sizeof(s_vehicle_identity.ecu_profile), "bmw-n47-dde7");
        profile_recognized = true;
    }
    /* Audi/Škoda entries below are manual-only candidates. VIN WMI narrows the
     * candidate family; it is deliberately not treated as an exact ECU match.
     */
    if (!profile_recognized && s_vehicle_identity.vin_known &&
        obd_vehicle_string_known(s_vehicle_identity.f197) &&
        obd_vehicle_vin_is_audi(s_vehicle_identity.vin)) {
        safe_copy_string(s_vehicle_identity.ecu_profile,
                         sizeof(s_vehicle_identity.ecu_profile), "audi-uds-powertrain-candidates");
        profile_recognized = true;
    }
    if (!profile_recognized && s_vehicle_identity.vin_known &&
        obd_vehicle_string_known(s_vehicle_identity.f197) &&
        obd_vehicle_vin_is_skoda(s_vehicle_identity.vin)) {
        safe_copy_string(s_vehicle_identity.ecu_profile,
                         sizeof(s_vehicle_identity.ecu_profile), "skoda-octavia-engine");
        profile_recognized = true;
    }
    if (!profile_recognized && obd_vehicle_string_known(s_vehicle_identity.f197)) {
        const char *f197 = s_vehicle_identity.f197;
        if (strstr(f197, "MED17") != NULL || strstr(f197, "MED 17") != NULL ||
            strstr(f197, "ME17") != NULL || strstr(f197, "MEB") != NULL ||
            strstr(f197, "VAG") != NULL) {
            safe_copy_string(s_vehicle_identity.ecu_profile, sizeof(s_vehicle_identity.ecu_profile), "vag-med17-generic");
            profile_recognized = true;
        }
    }
    if (!profile_recognized) {
        safe_copy_string(s_vehicle_identity.ecu_profile, sizeof(s_vehicle_identity.ecu_profile), "UNKNOWN");
    }
}

static void obd_vehicle_set_profile_match(void)
{
    obd_vehicle_apply_known_profile_from_fingerprint();

    const bool has_route = s_vehicle_identity.ecu_tx_id != 0U &&
                           s_vehicle_identity.ecu_rx_id != 0U;
    const bool has_f187 = obd_vehicle_string_known(s_vehicle_identity.f187);
    const bool has_f189 = obd_vehicle_string_known(s_vehicle_identity.f189);
    const bool has_f191 = obd_vehicle_string_known(s_vehicle_identity.f191);
    const bool has_f1a3 = obd_vehicle_string_known(s_vehicle_identity.f1a3);
    const bool has_f197 = obd_vehicle_string_known(s_vehicle_identity.f197);
    const bool has_f1ad = obd_vehicle_string_known(s_vehicle_identity.f1ad);
    const bool has_f19e = obd_vehicle_string_known(s_vehicle_identity.f19e);
    const bool has_f1a2 = obd_vehicle_string_known(s_vehicle_identity.f1a2);
    const bool has_calibration = obd_vehicle_string_known(s_vehicle_identity.calibration_id);
    const bool has_cvn = obd_vehicle_string_known(s_vehicle_identity.cvn);

    const bool vag_exact = has_route && s_vehicle_identity.vin_known &&
        has_f187 && has_f189 && has_f191 && has_f1a3 && has_f197 && has_f1ad &&
        has_f19e && has_f1a2 && has_calibration && has_cvn &&
        strcmp(s_vehicle_identity.ecu_profile, "vag-med17-generic") == 0 &&
        diag_ecu_profile_is_known(s_vehicle_identity.ecu_profile);

    /* BMW enablement is deliberately much narrower: only the documented N47 DDE7
     * profile on the observed physical 7E0/7E8 route. Other BMW engines remain partial. */
    const bool bmw_n47_exact = has_route && s_vehicle_identity.vin_known &&
        obd_vehicle_vin_is_bmw(s_vehicle_identity.vin) &&
        has_f187 && has_f189 && has_f191 && has_f197 &&
        s_vehicle_identity.ecu_tx_id == 0x7E0U &&
        s_vehicle_identity.ecu_rx_id == 0x7E8U &&
        strcmp(s_vehicle_identity.ecu_profile, "bmw-n47-dde7") == 0 &&
        diag_ecu_profile_is_known(s_vehicle_identity.ecu_profile);

    if (vag_exact || bmw_n47_exact) {
        s_vehicle_identity.match_level = OBD_VEHICLE_MATCH_EXACT;
        s_vehicle_identity.exact_ecu_match = true;
        s_vehicle_identity.auto_did_allowed = true;
        return;
    }

    const bool has_fingerprint = s_vehicle_identity.vin_known || has_route || has_f187 ||
        has_f189 || has_f191 || has_f1a3 || has_f197 || has_f1ad || has_f19e ||
        has_f1a2 || has_calibration || has_cvn;
    if (has_fingerprint) {
        s_vehicle_identity.match_level = OBD_VEHICLE_MATCH_PARTIAL;
        s_vehicle_identity.exact_ecu_match = false;
        s_vehicle_identity.auto_did_allowed = false;
        return;
    }
    s_vehicle_identity.match_level = OBD_VEHICLE_MATCH_UNKNOWN;
    s_vehicle_identity.exact_ecu_match = false;
    s_vehicle_identity.auto_did_allowed = false;
}

static void process_mode09_response(const can_frame_t *frame)
{
    if (frame == NULL || frame->rtr || frame->dlc < 3U || frame->data[0] != 0x49U) {
        return;
    }
    const uint8_t pid = frame->data[1];
    const uint8_t *payload = frame->data + 2U;
    const size_t payload_len = (size_t)frame->dlc - 2U;

    if (pid == 0x00U) {
        obd_vehicle_apply_mode09_bitmap(payload, payload_len);
        s_vehicle_identity.ecu_rx_id = frame->id;
        s_vehicle_identity.ecu_tx_id = s_can_config.extended_ids ? OBD_REQUEST_ID_EXTENDED : OBD_REQUEST_ID_STANDARD;
        s_vehicle_identification_pending = true;
        s_vehicle_identity.initialized = true;
        s_vehicle_identity.match_level = OBD_VEHICLE_MATCH_PARTIAL;
        ESP_LOGI(TAG, "Mode 09 support bitmap received from CAN ID 0x%0*lX",
                 frame->extended ? 8 : 3, (unsigned long)frame->id);
        if (obd_vehicle_mode09_supports_pid(0x02U)) {
            send_mode09_request(0x02U);
        }
        if (obd_vehicle_mode09_supports_pid(0x04U)) {
            send_mode09_request(0x04U);
        }
        if (obd_vehicle_mode09_supports_pid(0x06U)) {
            send_mode09_request(0x06U);
        }
        if (obd_vehicle_mode09_supports_pid(0x0AU)) {
            send_mode09_request(0x0AU);
        }
        return;
    }
    if (pid == 0x02U && payload_len > 0U) {
        char vin[18] = {0};
        const size_t max_vin = sizeof(vin) - 1U;
        for (size_t i = 0; i < payload_len && i < max_vin; ++i) {
            vin[i] = (char)payload[i];
        }
        if (obd_vehicle_vin_is_valid(vin)) {
            safe_copy_string(s_vehicle_identity.vin, sizeof(s_vehicle_identity.vin), vin);
            s_vehicle_identity.vin_known = true;
            obd_vehicle_apply_vag_year_from_vin(vin);
            s_vehicle_identity.match_level = OBD_VEHICLE_MATCH_PARTIAL;
            ESP_LOGI(TAG, "VIN decoded: %s (year=%s)", vin, s_vehicle_identity.model_year);
        } else {
            ESP_LOGW(TAG, "Received VIN payload is invalid or incomplete: %s", vin);
        }
        obd_vehicle_set_profile_match();
        return;
    }
    if (pid == 0x04U && payload_len > 0U) {
        char calibration[32] = {0};
        const size_t max_len = sizeof(calibration) - 1U;
        for (size_t i = 0; i < payload_len && i < max_len; ++i) {
            calibration[i] = (char)payload[i];
        }
        safe_copy_string(s_vehicle_identity.calibration_id, sizeof(s_vehicle_identity.calibration_id), calibration);
        ESP_LOGI(TAG, "Mode 09 calibration ID=%s", s_vehicle_identity.calibration_id);
        obd_vehicle_set_profile_match();
        return;
    }
    if (pid == 0x06U && payload_len > 0U) {
        char cvn[32] = {0};
        const size_t max_len = sizeof(cvn) - 1U;
        for (size_t i = 0; i < payload_len && i < max_len; ++i) {
            cvn[i] = (char)payload[i];
        }
        safe_copy_string(s_vehicle_identity.cvn, sizeof(s_vehicle_identity.cvn), cvn);
        ESP_LOGI(TAG, "Mode 09 CVN=%s", s_vehicle_identity.cvn);
        obd_vehicle_set_profile_match();
        return;
    }
    if (pid == 0x0AU && payload_len > 0U) {
        char ecu_name[32] = {0};
        const size_t max_len = sizeof(ecu_name) - 1U;
        for (size_t i = 0; i < payload_len && i < max_len; ++i) {
            ecu_name[i] = (char)payload[i];
        }
        if (ecu_name[0] != '\0') {
            ESP_LOGI(TAG, "Mode 09 ECU Name=%s", ecu_name);
        }
    }
}

static void send_mode09_request(uint8_t pid)
{
    const uint32_t request_id = s_can_config.extended_ids ? OBD_REQUEST_ID_EXTENDED : OBD_REQUEST_ID_STANDARD;
    can_frame_t tx = {
        .id = request_id,
        .extended = s_can_config.extended_ids,
        .rtr = false,
        .dlc = 3U,
        .data = {0U},
    };
    tx.data[0] = 0x02U;
    tx.data[1] = 0x09U;
    tx.data[2] = pid;
    esp_err_t err = can_send(&tx);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Mode 09 request 09 %02X failed: %s", pid, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Mode 09 request queued: 09 %02X on CAN ID 0x%0*lX",
             pid,
             tx.extended ? 8 : 3,
             (unsigned long)tx.id);
}

static void record_identity_response(const uds_event_t *event)
{
    if (event == NULL || event->type != UDS_EVENT_RESPONSE || event->service != 0x22U) {
        return;
    }
    if (event->data_length == 0U) {
        return;
    }

    char value[64] = {0};
    format_hex_bytes(event->data, event->data_length, value, sizeof(value));
    if (s_vehicle_identification_pending) {
        char raw_hex[256] = {0};
        char decoded[96] = {0};
        format_hex_bytes(event->data, event->data_length, raw_hex, sizeof(raw_hex));
        did_profile_decode_identity(event->did, event->data, event->data_length,
                                    decoded, sizeof(decoded));
        const uds_did_descriptor_t *info = uds_did_catalog_find(event->did, DIAG_ECU_ROLE_ENGINE);
        ESP_LOGI(TAG,
                 "[VEH_DECODE] DID=0x%04X NAME=\"%s\" DECODED=\"%s\" RAW_LEN=%u RAW=[%s]",
                 event->did, info != NULL && info->name != NULL ? info->name : "identification",
                 decoded, (unsigned)event->data_length, raw_hex);
        if (event->did == 0xF190U) {
            char vin[18] = {0};
            const size_t n = event->data_length < 17U ? event->data_length : 17U;
            memcpy(vin, event->data, n);
            const bool valid = obd_vehicle_vin_is_valid(vin);
            ESP_LOGI(TAG,
                     "[VEH_DECODE] VIN=%s VALID=%s WMI=%.3s BRAND=%s VDS=%.6s MODEL_YEAR_CHAR=%c PLANT=%c SERIAL=%.6s",
                     vin, valid ? "YES" : "NO", vin,
                     did_profile_brand_name(did_profile_brand_from_vin(vin)),
                     n >= 9U ? vin + 3 : "", n >= 10U ? vin[9] : '?',
                     n >= 11U ? vin[10] : '?', n >= 17U ? vin + 11 : "");
        }
    }
    switch (event->did) {
    case 0xF190U:
        if (event->data_length > 0U && event->data_length <= 17U) {
            char vin[18] = {0};
            size_t vin_len = (event->data_length < sizeof(vin) - 1U) ? event->data_length : (sizeof(vin) - 1U);
            for (size_t i = 0; i < vin_len; ++i) {
                vin[i] = (char)event->data[i];
            }
            if (obd_vehicle_vin_is_valid(vin)) {
                safe_copy_string(s_vehicle_identity.vin, sizeof(s_vehicle_identity.vin), vin);
                s_vehicle_identity.vin_known = true;
                obd_vehicle_apply_vag_year_from_vin(vin);
            }
        }
        break;
    case 0xF187U:
        safe_copy_string(s_vehicle_identity.f187, sizeof(s_vehicle_identity.f187), value);
        break;
    case 0xF189U:
        safe_copy_string(s_vehicle_identity.f189, sizeof(s_vehicle_identity.f189), value);
        break;
    case 0xF191U:
        safe_copy_string(s_vehicle_identity.f191, sizeof(s_vehicle_identity.f191), value);
        break;
    case 0xF1A3U:
        safe_copy_string(s_vehicle_identity.f1a3, sizeof(s_vehicle_identity.f1a3), value);
        break;
    case 0xF197U:
        obd_vehicle_copy_ascii(s_vehicle_identity.f197,
                               sizeof(s_vehicle_identity.f197),
                               event->data, event->data_length);
        break;
    case 0xF1ADU:
        safe_copy_string(s_vehicle_identity.f1ad, sizeof(s_vehicle_identity.f1ad), value);
        break;
    case 0xF19EU:
        safe_copy_string(s_vehicle_identity.f19e, sizeof(s_vehicle_identity.f19e), value);
        break;
    case 0xF1A2U:
        safe_copy_string(s_vehicle_identity.f1a2, sizeof(s_vehicle_identity.f1a2), value);
        break;
    case 0xF1AAU:
        safe_copy_string(s_vehicle_identity.f1aa, sizeof(s_vehicle_identity.f1aa), value);
        break;
    case 0xF17CU:
        safe_copy_string(s_vehicle_identity.f17c, sizeof(s_vehicle_identity.f17c), value);
        break;
    case 0xF1F4U:
        safe_copy_string(s_vehicle_identity.f1f4, sizeof(s_vehicle_identity.f1f4), value);
        break;
    case 0xF1A5U:
        safe_copy_string(s_vehicle_identity.f1a5, sizeof(s_vehicle_identity.f1a5), value);
        break;
    case 0xF804U:
        safe_copy_string(s_vehicle_identity.calibration_id,
                         sizeof(s_vehicle_identity.calibration_id), value);
        break;
    case 0xF806U:
        safe_copy_string(s_vehicle_identity.cvn, sizeof(s_vehicle_identity.cvn), value);
        break;
    default:
        break;
    }
    if (s_uds_channel_used[0]) {
        s_vehicle_identity.ecu_tx_id = s_uds[0].transport.config.tx_id;
        s_vehicle_identity.ecu_rx_id = s_uds[0].transport.config.rx_id;
    }
    obd_vehicle_set_profile_match();
}

static bool obd_client_any_uds_transaction_active(void)
{
    for (uint8_t i = 0U; i < OBD_ECU_COUNT; ++i) {
        if (s_uds_channel_used[i] && uds_client_transaction_active(&s_uds[i])) {
            return true;
        }
    }
    return false;
}

static bool obd_client_vehicle_identity_did_is_queried_by_v(uint16_t did)
{
    for (size_t i = 0U;
         i < sizeof(s_vehicle_identity_dids) / sizeof(s_vehicle_identity_dids[0]);
         ++i) {
        if (s_vehicle_identity_dids[i] == did) {
            return true;
        }
    }
    return false;
}

static void queue_vehicle_identity_dids(void)
{
    const size_t did_count = sizeof(s_vehicle_identity_dids) /
                             sizeof(s_vehicle_identity_dids[0]);

    if (!s_vehicle_identification_pending || !s_polling_enabled ||
        s_waiting_for_response || !s_uds_channel_used[0] ||
        uds_client_transaction_active(&s_uds[0])) {
        return;
    }

    if (s_vehicle_identity_queue_index == SIZE_MAX ||
        did_profile_identity_did_at(s_vehicle_identity_queue_index,
                                    s_vehicle_identity_dids, did_count) == 0U) {
        s_vehicle_identification_pending = false;
        s_vehicle_identification_started = false;
        obd_vehicle_set_profile_match();
        ESP_LOGI(TAG,
                 "[VEH] ECU identity sequence complete: VIN=%s profile=%s match=%s",
                 s_vehicle_identity.vin,
                 s_vehicle_identity.ecu_profile,
                 s_vehicle_identity.match_level == OBD_VEHICLE_MATCH_EXACT ? "EXACT" :
                 (s_vehicle_identity.match_level == OBD_VEHICLE_MATCH_PARTIAL ? "PARTIAL" : "UNKNOWN"));
        did_poll_arm_after_identity();
        return;
    }

    const uint16_t did = did_profile_identity_did_at(
        s_vehicle_identity_queue_index, s_vehicle_identity_dids, did_count);
    if (did == 0U) {
        s_vehicle_identity_queue_index = SIZE_MAX; /* sequence finished */
        return;
    }
    const esp_err_t err = obd_client_uds_read_did(0U, did);
    if (err == ESP_OK) {
        ++s_vehicle_identity_queue_index;
        ESP_LOGI(TAG, "[VEH] Queued ECU identity DID 0x%04X", did);
    } else {
        ESP_LOGW(TAG, "[VEH] ECU identity DID 0x%04X could not be queued: %s",
                 did, esp_err_to_name(err));
    }
}

static esp_err_t vehicle_identify_begin(bool restart_bitmap, bool quiet)
{
    if (!s_started || !(OBD_ACTIVE_TX_ENABLED || OBD_LOOPBACK_SIM_ENABLED)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_polling_enabled || s_can_discovery.phase != CAN_DISCOVERY_LOCKED ||
        !s_uds_channel_used[0]) {
        if (!quiet) ESP_LOGW(TAG,
                 "[VEH] Identity query refused: waiting for a verified CAN link and physical ECU channel");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_vehicle_identification_pending || s_did_scan.active ||
        obd_client_any_uds_transaction_active()) {
        if (!quiet) ESP_LOGW(TAG, "[VEH] Identity query refused: another diagnostic transaction is active");
        return ESP_ERR_INVALID_STATE;
    }

    const isotp_config_t *route = &s_uds[0].transport.config;
    const diag_ecu_role_t role = diag_ecu_role_for_rx_id(route->rx_id, route->extended_id);
    if ((role != DIAG_ECU_ROLE_ENGINE && role != DIAG_ECU_ROLE_TRANSMISSION) ||
        !diag_ecu_route_is_allowed(route->tx_id, route->rx_id,
                                   route->extended_id, route->extended_id, role)) {
        ESP_LOGW(TAG,
                 "[VEH] Identity query refused: ECU route TX=0x%lX RX=0x%lX is not an enabled read-only route",
                 (unsigned long)route->tx_id, (unsigned long)route->rx_id);
        return ESP_ERR_NOT_SUPPORTED;
    }

    obd_vehicle_reset_identity_state();
    s_vehicle_identity.initialized = true;
    s_vehicle_identity.match_level = OBD_VEHICLE_MATCH_PARTIAL;
    s_vehicle_identity.request_tx_id = route->tx_id;
    s_vehicle_identity.response_rx_id = route->rx_id;
    s_vehicle_identity.ecu_tx_id = route->tx_id;
    s_vehicle_identity.ecu_rx_id = route->rx_id;
    s_vehicle_identity.auto_did_allowed = false;
    s_vehicle_identity.exact_ecu_match = false;
    safe_copy_string(s_vehicle_identity.ecu_profile,
                     sizeof(s_vehicle_identity.ecu_profile), "UNKNOWN");

    s_vehicle_identity_queue_index = 0U;
    s_vehicle_identification_started = true;
    s_vehicle_identification_pending = true;
    did_poll_reset();
#if OBD_SCHEDULER_ENABLED
    if (restart_bitmap) {
        /* Manual `v`: re-ask the PID bitmap together with the identity sequence. */
        s_discovery_base = OBD_DISCOVERY_PID;
        s_discovery_complete = false;
        s_discovery_response_seen = false;
        s_discovery_has_continuation = false;
        s_discovery_collect_until_us = 0;
        s_next_request_us = esp_timer_get_time();
    }
#endif
    (void)restart_bitmap;

    ESP_LOGI(TAG,
             "[VEH] Starting read-only UDS identity sequence (0x22 only, %s) on verified ECU RX=0x%0*lX bitrate=%lu kbit/s",
             restart_bitmap ? "manual" : "automatic",
             route->extended_id ? 8 : 3, (unsigned long)route->rx_id,
             (unsigned long)(s_can_config.bitrate / 1000U));
    return ESP_OK;
}

esp_err_t obd_client_vehicle_identify(void)
{
    return vehicle_identify_begin(true, false);
}

/* Runs the identity (VEH) layer once per locked CAN link, alongside the PID bitmap. */
static void vehicle_identify_auto(void)
{
    if (s_can_discovery.phase != CAN_DISCOVERY_LOCKED) {
        s_auto_identity_attempted = false;
        return;
    }
    if (s_auto_identity_attempted || !s_uds_channel_used[0]) return;
    const esp_err_t err = vehicle_identify_begin(false, true);
    if (err == ESP_ERR_INVALID_STATE) return; /* busy: retry on a later pass */
    s_auto_identity_attempted = true;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[VEH] Automatic identity skipped: %s", esp_err_to_name(err));
    }
}

const obd_vehicle_identity_t *obd_client_vehicle_identity(void)
{
    return &s_vehicle_identity;
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

static esp_err_t initialize_uds_channel(uint8_t index, uint32_t rx_id)
{
    if (index >= OBD_ECU_COUNT) return ESP_ERR_INVALID_ARG;

    const uint32_t tx_id = s_can_config.extended_ids
        ? 0x18DA0000U | ((rx_id & 0xFFU) << 8) | 0xF1U
        : 0x7E0U + (rx_id - OBD_RESPONSE_ID_FIRST);
    const isotp_config_t config = {
        .tx_id = tx_id,
        .rx_id = rx_id,
        .extended_id = s_can_config.extended_ids,
        .padding = 0x00U,
        .rx_block_size = 0U,
        .rx_st_min = 0U,
    };
    esp_err_t err = uds_client_init(&s_uds[index], &config,
                                    send_isotp_frame, NULL);
    if (err != ESP_OK) return err;
    s_uds_rx_ids[index] = rx_id;
    s_uds_channel_used[index] = true;
    return ESP_OK;
}

static esp_err_t initialize_isotp_channels(void)
{
    esp_err_t err = can_get_config(&s_can_config);
    if (err != ESP_OK) return err;

    memset(s_uds, 0, sizeof(s_uds));
    memset(s_uds_rx_ids, 0, sizeof(s_uds_rx_ids));
    memset(s_uds_channel_used, 0, sizeof(s_uds_channel_used));

    if (!s_can_config.extended_ids) {
        for (uint8_t index = 0U; index < OBD_ECU_COUNT; ++index) {
            err = initialize_uds_channel(index, OBD_RESPONSE_ID_FIRST + index);
            if (err != ESP_OK) return err;
        }
    }
    return ESP_OK;
}

static int find_or_create_uds_channel(const can_frame_t *frame)
{
    if (frame == NULL || !is_obd_response_id(frame)) return -1;

    if (s_vehicle_identification_started && s_vehicle_identity.ecu_rx_id == 0U) {
        s_vehicle_identity.ecu_rx_id = frame->id;
        s_vehicle_identity.ecu_tx_id = s_can_config.extended_ids ? OBD_REQUEST_ID_EXTENDED : OBD_REQUEST_ID_STANDARD;
    }

    if (!frame->extended) {
        const uint8_t index = (uint8_t)(frame->id - OBD_RESPONSE_ID_FIRST);
        return index < OBD_ECU_COUNT && s_uds_channel_used[index] ? (int)index : -1;
    }

    int unused = -1;
    for (uint8_t index = 0U; index < OBD_ECU_COUNT; ++index) {
        if (s_uds_channel_used[index] && s_uds_rx_ids[index] == frame->id) {
            return (int)index;
        }
        if (!s_uds_channel_used[index] && unused < 0) unused = (int)index;
    }
    if (unused < 0) {
        ESP_LOGW(TAG, "Ignoring extended OBD ECU 0x%08lX: channel capacity (%u) reached",
                 (unsigned long)frame->id, (unsigned)OBD_ECU_COUNT);
        return -1;
    }
    esp_err_t err = initialize_uds_channel((uint8_t)unused, frame->id);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not allocate ISO-TP channel for ECU 0x%08lX: %s",
                 (unsigned long)frame->id, esp_err_to_name(err));
        return -1;
    }
    return unused;
}

static esp_err_t begin_can_candidate_listen(int64_t now_us)
{
    const can_bus_config_t *candidate = can_discovery_candidate(&s_can_discovery);
    if (candidate == NULL) return ESP_ERR_NOT_FOUND;

    esp_err_t err = can_set_mode(CAN_MODE_CONFIG);
    if (err != ESP_OK) return err;
    err = can_reconfigure(candidate);
    if (err != ESP_OK) return err;
    err = can_configure_diagnostic_accept_all_rx();
    if (err != ESP_OK) return err;
    err = can_set_mode(CAN_MODE_LISTEN_ONLY);
    if (err != ESP_OK) return err;
    bool stale_message_error = false;
    err = can_read_and_clear_message_error(&stale_message_error);
    if (err != ESP_OK) return err;
    if (stale_message_error) {
        ESP_LOGW(TAG, "Cleared a stale MCP2515 message-error flag before listen window");
    }
    err = can_get_config(&s_can_config);
    if (err != ESP_OK) return err;

    s_polling_enabled = false;
    s_waiting_for_response = false;
    s_requested_pid = 0U;
    s_candidate_deadline_us = now_us + CAN_DISCOVERY_LISTEN_US;
    ESP_LOGI(TAG, "Passive CAN candidate: %lu kbit/s %s; listening for valid frames",
             (unsigned long)(s_can_config.bitrate / 1000U),
             s_can_config.extended_ids ? "29-bit" : "11-bit");
    return ESP_OK;
}

#if OBD_ACTIVE_TX_ENABLED && !OBD_LOOPBACK_SIM_ENABLED
static esp_err_t begin_can_candidate_verification(int64_t now_us)
{
    esp_err_t err = can_discovery_begin_verification(&s_can_discovery);
    if (err != ESP_OK) return err;
    err = can_set_mode(CAN_MODE_CONFIG);
    if (err != ESP_OK) return err;
    err = can_configure_obd_response_filters();
    if (err != ESP_OK) return err;
    err = initialize_isotp_channels();
    if (err != ESP_OK) return err;
    err = can_set_mode(CAN_MODE_NORMAL);
    if (err != ESP_OK) return err;

    s_polling_enabled = true;
    s_next_request_us = now_us;
    s_verify_timeouts = 0U;
    ESP_LOGI(TAG, "Bus alive at %lu kbit/s (%s IDs); probing OBD with 01 00",
             (unsigned long)(s_can_config.bitrate / 1000U),
             s_can_config.extended_ids ? "29-bit" : "11-bit");
    onx_display_show_waiting_for_rpm("VERIFYING OBD");
    return ESP_OK;
}
#endif

/* All candidates exhausted: stop polling and leave the controller in
 * Configuration mode while waiting for the rescan, so a wrong-bitrate bus does
 * not flood the RX buffers and the controller does not ACK in Normal mode. */
static void enter_discovery_idle(int64_t now_us)
{
    s_polling_enabled = false;
    s_waiting_for_response = false;
    s_requested_pid = 0U;
    s_discovery_retry_us = now_us + CAN_DISCOVERY_RETRY_US;
    esp_err_t err = can_set_mode(CAN_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not enter Configuration mode while idle: %s",
                 esp_err_to_name(err));
    }
}

static void reject_current_can_candidate(const char *reason, int64_t now_us)
{
    ESP_LOGW(TAG, "CAN candidate rejected: %s", reason);
    can_discovery_reject_candidate(&s_can_discovery);
    s_can_discovery_enabled = true;
    if (can_discovery_candidate(&s_can_discovery) == NULL) {
        enter_discovery_idle(now_us);
        ESP_LOGW(TAG, "No candidate returned a valid 41 00 bitmap; will passively rescan");
        onx_display_show_waiting_for_rpm("NO OBD RESPONSE");
        return;
    }
    esp_err_t err = begin_can_candidate_listen(now_us);
    if (err != ESP_OK) {
        s_polling_enabled = false;
        s_can_discovery_enabled = false;
        ESP_LOGE(TAG, "Could not start next CAN candidate: %s", esp_err_to_name(err));
    }
}

static void poll_can_discovery(int64_t now_us)
{
    if (!s_can_discovery_enabled) return;
    if (s_can_discovery.phase == CAN_DISCOVERY_LISTENING) {
        bool message_error = false;
        esp_err_t err = can_read_and_clear_message_error(&message_error);
        if (err != ESP_OK) {
            s_polling_enabled = false;
            s_can_discovery_enabled = false;
            ESP_LOGE(TAG, "Cannot verify MCP2515 message-error flag; refusing active probing: %s",
                     esp_err_to_name(err));
            return;
        }
        if (message_error) {
            reject_current_can_candidate(
                "MCP2515 reported a CAN message error in Listen-Only mode", now_us);
            return;
        }
    }
    if (s_can_discovery.phase == CAN_DISCOVERY_EXHAUSTED) {
        if (s_discovery_retry_us != 0 && now_us >= s_discovery_retry_us) {
            can_discovery_restart(&s_can_discovery);
            s_discovery_retry_us = 0;
            esp_err_t err = begin_can_candidate_listen(now_us);
            if (err != ESP_OK) {
                s_can_discovery_enabled = false;
                ESP_LOGE(TAG, "Could not restart CAN discovery: %s", esp_err_to_name(err));
            }
        }
        return;
    }
    if (s_can_discovery.phase != CAN_DISCOVERY_LISTENING ||
        now_us < s_candidate_deadline_us) {
        return;
    }

    if (can_discovery_has_evidence(&s_can_discovery)) {
#if OBD_ACTIVE_TX_ENABLED && !OBD_LOOPBACK_SIM_ENABLED
        esp_err_t err = begin_can_candidate_verification(now_us);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Could not verify CAN candidate: %s", esp_err_to_name(err));
            reject_current_can_candidate("candidate configuration failed", now_us);
        }
#else
        if (can_discovery_lock_passive(&s_can_discovery) == ESP_OK) {
            s_can_discovery_enabled = false;
            ESP_LOGI(TAG, "Passive CAN detected: %lu kbit/s %s; OBD requests remain disabled",
                     (unsigned long)(s_can_config.bitrate / 1000U),
                     s_can_config.extended_ids ? "29-bit" : "11-bit");
            onx_display_show_waiting_for_rpm("CAN DETECTED - PASSIVE");
        }
#endif
        return;
    }

    can_discovery_reject_candidate(&s_can_discovery);
    if (can_discovery_candidate(&s_can_discovery) == NULL) {
        enter_discovery_idle(now_us);
        ESP_LOGW(TAG, "No valid CAN frames on 250/500 kbit/s, 11/29-bit candidates; scan is inconclusive");
        onx_display_show_waiting_for_rpm("NO CAN TRAFFIC");
        return;
    }
    esp_err_t err = begin_can_candidate_listen(now_us);
    if (err != ESP_OK) {
        s_can_discovery_enabled = false;
        ESP_LOGE(TAG, "Could not start next CAN candidate: %s", esp_err_to_name(err));
    }
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
    if (!s_uds_channel_used[ecu_index]) return ESP_ERR_NOT_FOUND;
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
                if (matching_request) {
                    record_pid_error(pid, OBD_PID_ERROR_INVALID_RESPONSE);
                    if (s_can_discovery.phase == CAN_DISCOVERY_VERIFYING) {
                        reject_current_can_candidate("malformed 41 00 bitmap", now_us);
                    }
                    record_miss("Malformed supported-PID bitmap");
                }
                return;
            }
            const bool verifying_bitmap =
                s_can_discovery.phase == CAN_DISCOVERY_VERIFYING;
            if (verifying_bitmap && !matching_request) return;
            esp_err_t err = parameter_manager_record_supported_page(
                frame->id, frame->extended, pid, &payload[2]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Could not cache ECU PID bitmap: %s", esp_err_to_name(err));
                if (verifying_bitmap) {
                    reject_current_can_candidate("invalid PID bitmap", now_us);
                }
                return;
            }
            if (verifying_bitmap) {
                if (!can_discovery_confirm(&s_can_discovery, frame, payload, length)) {
                    reject_current_can_candidate("invalid 41 00 response address", now_us);
                    return;
                }
                s_can_discovery_enabled = false;
                ESP_LOGI(TAG,
                         "CAN/OBD verified by ECU 0x%0*lX: %lu kbit/s %s and valid 41 00 bitmap",
                         frame->extended ? 8 : 3, (unsigned long)frame->id,
                         (unsigned long)(s_can_config.bitrate / 1000U),
                         s_can_config.extended_ids ? "29-bit" : "11-bit");
                onx_display_set_status("OBD CONNECTED");
            }
            s_discovery_response_seen = true;
            s_discovery_has_continuation |=
                parameter_manager_page_has_continuation(&payload[2]);
            refresh_dashboard_catalog();
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
                if (matching_request) {
                    record_pid_error(pid, OBD_PID_ERROR_INVALID_RESPONSE);
                    onx_display_update_parameter(pid, 0.0f, PARAM_VALIDITY_INVALID,
                                                 now_us);
                    record_miss("Malformed or out-of-range OBD response");
                }
                return;
            }
            if (!s_pid_source_valid[pid]) {
                s_pid_source_valid[pid] = true;
                s_pid_source_id[pid] = frame->id;
                s_pid_source_extended[pid] = frame->extended;
            }
            const bool selected_source =
                s_pid_source_id[pid] == frame->id &&
                s_pid_source_extended[pid] == frame->extended;
            if (selected_source) {
                s_pid_status[pid].last_success_us = now_us;
                s_pid_status[pid].last_error = OBD_PID_ERROR_NONE;
                onx_display_update_parameter(pid, value.value, value.validity, now_us);

                if (pid == OBD_PID_ENGINE_RPM) {
                    const uint32_t rpm = (uint32_t)(value.value + 0.5f);
                    ++s_successful_rpm_responses;
                    ESP_LOGI(TAG, "Engine RPM: %lu rpm (ECU CAN ID 0x%0*lX; response #%lu)",
                             (unsigned long)rpm, frame->extended ? 8 : 3,
                             (unsigned long)frame->id,
                             (unsigned long)s_successful_rpm_responses);
                } else if (pid == OBD_PID_COOLANT_TEMP) {
                    const int32_t temp_c = (int32_t)(value.value + 0.5f);
                    ++s_successful_coolant_responses;
                    ESP_LOGI(TAG, "Coolant temperature: %ld C (ECU CAN ID 0x%0*lX; response #%lu)",
                             (long)temp_c, frame->extended ? 8 : 3,
                             (unsigned long)frame->id,
                             (unsigned long)s_successful_coolant_responses);
                } else {
                    size_t count = 0U;
                    const parameter_descriptor_t *catalog =
                        parameter_manager_catalog(&count);
                    for (size_t i = 0; i < count; ++i) {
                        if (catalog[i].pid == pid) {
                            ESP_LOGI(TAG, "%s: %.2f %s (ECU CAN ID 0x%0*lX; raw length=%u)",
                                     catalog[i].name, (double)value.value, catalog[i].unit,
                                     frame->extended ? 8 : 3,
                                     (unsigned long)frame->id, value.raw_length);
                            if (value.secondary_valid) {
                                ESP_LOGI(TAG, "%s secondary channel: %.3f",
                                         catalog[i].name, (double)value.secondary_value);
                            }
                            break;
                        }
                    }
                }
                onx_display_set_status("OBD DATA LIVE");
            } else {
                ESP_LOGD(TAG, "Ignoring duplicate PID 0x%02X value from ECU 0x%0*lX; "
                         "display source remains 0x%0*lX",
                         pid, frame->extended ? 8 : 3, (unsigned long)frame->id,
                         s_pid_source_extended[pid] ? 8 : 3,
                         (unsigned long)s_pid_source_id[pid]);
            }
        }

        if (matching_request) {
            s_waiting_for_response = false;
            s_requested_pid = 0U;
            s_consecutive_misses = 0U;
#if OBD_SCHEDULER_ENABLED
            s_next_request_us = now_us;
#endif
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
        if (s_waiting_for_response && payload[1] == OBD_MODE_CURRENT_DATA) {
            const uint8_t requested_pid = s_requested_pid;
            record_pid_error(s_requested_pid, OBD_PID_ERROR_NEGATIVE_RESPONSE);
            onx_display_update_parameter(requested_pid, 0.0f,
                                         PARAM_VALIDITY_INVALID,
                                         esp_timer_get_time());
            record_miss("ECU returned a negative diagnostic response");
            if (s_can_discovery.phase == CAN_DISCOVERY_VERIFYING) {
                reject_current_can_candidate("negative response to 01 00", esp_timer_get_time());
            }
        }
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

    if (s_can_discovery_enabled &&
        s_can_discovery.phase == CAN_DISCOVERY_LISTENING) {
        (void)can_discovery_observe(&s_can_discovery, frame);
        return;
    }

    if (!frame->rtr && frame->dlc >= 3U && frame->data[0] == 0x49U) {
        process_mode09_response(frame);
    }

    if (frame->rtr || !is_obd_response_id(frame)) return;

    const int channel = find_or_create_uds_channel(frame);
    if (channel < 0) return;

    const uds_event_type_t uds_result = uds_client_on_frame(
        &s_uds[channel], frame, (uint64_t)esp_timer_get_time(), &s_uds_event);
    const isotp_event_t *transport_event = &s_uds[channel].transport_event;
    if (transport_event->type == ISOTP_EVENT_PDU_RECEIVED &&
        transport_event->length >= 2U &&
        (transport_event->data[0] == 0x41U ||
         (transport_event->data[0] == 0x7FU &&
          transport_event->length >= 2U &&
          transport_event->data[1] == OBD_MODE_CURRENT_DATA))) {
        record_obd_response(frame, transport_event->data,
                            transport_event->length);
    }
    if (uds_result != UDS_EVENT_NONE) {
        log_uds_event((uint8_t)channel, &s_uds_event);
        record_identity_response(&s_uds_event);
        /* Feed frame-arrival responses to the active OEM scan immediately.
         * obd_client_poll() handles timeout events separately through tick(). */
        if (s_did_scan.active && (uint8_t)channel == s_did_scan.ecu_index) {
            obd_client_did_scan_handle_event((uint8_t)channel, &s_uds_event);
        }
        did_poll_handle_event((uint8_t)channel, &s_uds_event);
    }
}

static bool expire_obd_request(int64_t now_us)
{
    if (!s_waiting_for_response ||
        (now_us - s_request_sent_us) < OBD_RESPONSE_TIMEOUT_US) {
        return false;
    }

    const uint8_t timed_out_pid = s_requested_pid;
    parameter_manager_record_timeout(timed_out_pid, now_us);
    record_pid_error(timed_out_pid, OBD_PID_ERROR_TIMEOUT);
    onx_display_update_parameter(timed_out_pid, 0.0f,
                                 PARAM_VALIDITY_TIMEOUT, now_us);
    ESP_LOGW(TAG, "OBD Mode 01 PID 0x%02X response timeout after 200 ms",
             timed_out_pid);
    record_miss("No response for outstanding OBD request");
    if (s_can_discovery.phase == CAN_DISCOVERY_VERIFYING) {
        if (++s_verify_timeouts < CAN_DISCOVERY_VERIFY_ATTEMPTS) {
            ESP_LOGW(TAG, "No 41 00 yet (attempt %u/%u); retrying once before rejecting",
                     (unsigned)s_verify_timeouts, (unsigned)CAN_DISCOVERY_VERIFY_ATTEMPTS);
        } else {
            reject_current_can_candidate("timeout waiting for valid 41 00 bitmap", now_us);
        }
    }
    return true;
}

#if defined(CONFIG_OBD_RUN_PROTOCOL_TESTS) && CONFIG_OBD_RUN_PROTOCOL_TESTS
bool obd_client_test_process_response(uint8_t requested_pid,
                                      const uint8_t *payload, size_t length)
{
    if (payload == NULL) return false;
    memset(&s_pid_status[requested_pid], 0, sizeof(s_pid_status[requested_pid]));
    s_waiting_for_response = true;
    s_requested_pid = requested_pid;
    s_request_sent_us = esp_timer_get_time();
    const can_frame_t frame = {
        .id = OBD_RESPONSE_ID_FIRST,
        .dlc = 8U,
        .extended = false,
        .rtr = false,
    };
    record_obd_response(&frame, payload, length);
    return !s_waiting_for_response;
}

bool obd_client_test_expire_request(uint8_t requested_pid, int64_t elapsed_us)
{
    memset(&s_pid_status[requested_pid], 0, sizeof(s_pid_status[requested_pid]));
    s_waiting_for_response = true;
    s_requested_pid = requested_pid;
    s_request_sent_us = esp_timer_get_time();
    return expire_obd_request(s_request_sent_us + elapsed_us);
}
#endif

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

#if OBD_SCHEDULER_ENABLED
static uint8_t select_next_pid(int64_t now_us)
{
    if (!s_discovery_complete) return s_discovery_base;

    size_t count = 0;
    const parameter_descriptor_t *catalog = parameter_manager_catalog(&count);
    bool supported[256] = {false};
    for (size_t i = 0; i < count; ++i) {
        supported[catalog[i].pid] =
            parameter_manager_any_ecu_supports(catalog[i].pid);
    }
    return pid_scheduler_select(catalog, count, supported, s_next_due_us, now_us);
}
#endif

static void format_hex_bytes(const uint8_t *data, size_t length, char *buffer,
                             size_t buffer_len)
{
    if (buffer == NULL || buffer_len == 0U) return;
    size_t used = 0U;
    buffer[0] = '\0';
    if (data == NULL || length == 0U) return;
    for (size_t i = 0; i < length; ++i) {
        int written = snprintf(buffer + used, buffer_len - used,
                               (i == 0U) ? "%02X" : " %02X", data[i]);
        if (written < 0 || (size_t)written >= buffer_len - used) {
            break;
        }
        used += (size_t)written;
    }
}

static const char *nrc_name(uint8_t nrc)
{
    switch (nrc) {
    case 0x10U: return "GENERAL_REJECT";
    case 0x11U: return "SERVICE_NOT_SUPPORTED";
    case 0x12U: return "SUBFUNCTION_NOT_SUPPORTED";
    case 0x13U: return "INCORRECT_MESSAGE_LENGTH_OR_INVALID_FORMAT";
    case 0x14U: return "RESPONSE_TOO_LONG";
    case 0x21U: return "BUSY_REPEAT_REQUEST";
    case 0x22U: return "CONDITIONS_NOT_CORRECT";
    case 0x24U: return "REQUEST_SEQUENCE_ERROR";
    case 0x25U: return "NO_RESPONSE_FROM_SUBNET_COMPONENT";
    case 0x26U: return "FAILURE_PREVENTS_EXECUTION_OF_REQUEST";
    case 0x31U: return "REQUEST_OUT_OF_RANGE";
    case 0x33U: return "SECURITY_ACCESS_DENIED";
    case 0x35U: return "INVALID_KEY";
    case 0x36U: return "EXCEED_NUMBER_OF_ATTEMPTS";
    case 0x37U: return "REQUIRED_TIME_DELAY_NOT_EXPIRED";
    case 0x78U: return "RESPONSE_PENDING";
    default: return "UNKNOWN";
    }
}

static void log_oem_did_scan_summary(void)
{
    const uint32_t total = s_did_scan.total_count;
    const uint32_t positive = s_did_scan.positive_count;
    const uint32_t negative = s_did_scan.negative_count;
    const uint32_t timeout = s_did_scan.timeout_count;
    ESP_LOGI(TAG, "[OEM_DID_SCAN] SUMMARY total=%lu positive=%lu negative=%lu timeout=%lu",
             (unsigned long)total,
             (unsigned long)positive,
             (unsigned long)negative,
             (unsigned long)timeout);
}

static size_t obd_client_did_build_request(const uds_did_descriptor_t *descriptor,
                                             uint8_t request[4])
{
    if (descriptor == NULL || request == NULL) return 0U;
    switch (descriptor->service) {
    case 0x22U:
        request[0] = 0x22U;
        request[1] = (uint8_t)(descriptor->did >> 8);
        request[2] = (uint8_t)descriptor->did;
        return 3U;
    case 0x21U:
        if (descriptor->did > 0xFFU) return 0U;
        request[0] = 0x21U;
        request[1] = (uint8_t)descriptor->did;
        return 2U;
    case 0x2CU:
        if (descriptor->subfunction == 0U) return 0U;
        request[0] = 0x2CU;
        request[1] = descriptor->subfunction;
        request[2] = (uint8_t)(descriptor->did >> 8);
        request[3] = (uint8_t)descriptor->did;
        return 4U;
    default:
        return 0U;
    }
}

static esp_err_t obd_client_did_send_descriptor(uint8_t ecu_index,
                                                const uds_did_descriptor_t *descriptor,
                                                uint64_t now_us)
{
    if (ecu_index >= OBD_ECU_COUNT || descriptor == NULL) return ESP_ERR_INVALID_ARG;
    switch (descriptor->service) {
    case 0x22U:
        return uds_client_read_did(&s_uds[ecu_index], descriptor->did, now_us);
    case 0x21U:
        if (descriptor->did > 0xFFU) return ESP_ERR_INVALID_ARG;
        return uds_client_read_local_identifier(&s_uds[ecu_index],
                                                (uint8_t)descriptor->did, now_us);
    case 0x2CU:
        return uds_client_read_bmw_local_identifier(&s_uds[ecu_index],
                                                    descriptor->did, now_us);
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static void obd_client_did_scan_handle_event(uint8_t ecu_index,
                                            const uds_event_t *event)
{
    if (!s_did_scan.active || s_did_scan.state != OBD_DID_SCAN_STATE_RUNNING ||
        ecu_index != s_did_scan.ecu_index || event == NULL) {
        return;
    }
    if (s_did_scan.queue_index >= s_did_scan.queue_count) {
        return;
    }

    const uds_did_descriptor_t *descriptor = s_did_scan.queue[s_did_scan.queue_index];
    if (descriptor == NULL) {
        return;
    }
    const bool extended = s_can_config.extended_ids;
    const uint32_t tx_can_id = s_uds[ecu_index].transport.config.tx_id;
    const uint32_t rx_can_id = s_uds[ecu_index].transport.config.rx_id;
    const int64_t duration_ms = (esp_timer_get_time() - s_did_scan.request_sent_us) / 1000LL;
    uint8_t tx_request[4] = {0};
    const size_t tx_request_length = obd_client_did_build_request(descriptor, tx_request);

    if (event->type == UDS_EVENT_RESPONSE) {
        if (event->service == descriptor->service && event->did == descriptor->did) {
            s_did_scan.positive_count += 1U;
            s_did_scan.total_count += 1U;
            if (s_dp.set.count > 0U &&
                !did_poll_contains(&s_dp.set, descriptor->did, descriptor->service)) {
                ESP_LOGW(TAG,
                         "[DID_OUT_OF_PROFILE] DID=0x%04X SERVICE=0x%02X NAME=\"%s\" positive but not in the active profile poll set",
                         descriptor->did, descriptor->service,
                         descriptor->name != NULL ? descriptor->name : "unknown");
            }
            uint8_t response_bytes[UDS_CLIENT_MAX_DATA] = {0};
            memcpy(response_bytes, event->data, event->data_length);
            char response_hex[256] = {0};
            format_hex_bytes(response_bytes, event->data_length, response_hex, sizeof(response_hex));
            char request_hex[64] = {0};
            format_hex_bytes(tx_request, tx_request_length, request_hex, sizeof(request_hex));
            ESP_LOGI(TAG,
                     "[OEM_DID_SCAN] TIME=%lld ECU=%s CAN_FORMAT=%s BITRATE=%lu TX_CAN_ID=0x%0*lX RX_CAN_ID=0x%0*lX SERVICE=0x%02X IDENTIFIER=0x%04X TX_UDS=[%s] RX_UDS=[%s] STATUS=POSITIVE DATA_LENGTH=%u DATA=[%s] DECODER=%s DURATION_MS=%lld",
                     (long long)(esp_timer_get_time() / 1000LL),
                     oem_did_scan_role_name(s_did_scan.role),
                     extended ? "29BIT" : "11BIT",
                     (unsigned long)(s_can_config.bitrate / 1000U),
                     extended ? 8 : 3,
                     (unsigned long)tx_can_id,
                     extended ? 8 : 3,
                     (unsigned long)rx_can_id,
                     descriptor->service,
                     descriptor->did,
                     request_hex,
                     response_hex,
                     (unsigned)event->data_length,
                     response_hex,
                     descriptor->decoder != NULL ? descriptor->decoder : "raw only",
                     (long long)duration_ms);
            obd_client_did_scan_append_record("positive", descriptor->did, tx_request,
                                              tx_request_length, event->data,
                                              event->data_length, "POSITIVE", 0U,
                                              event->data_length > 7U, duration_ms);
            double decoded_value = 0.0;
            if (uds_did_catalog_decode(descriptor, event->data,
                                       event->data_length, &decoded_value)) {
                ESP_LOGI(TAG,
                         "[OEM_DID_DECODE] DID=0x%04X NAME=\"%s\" VALUE=%.6f UNIT=%s DECODE_KIND=%u FORMULA=raw*%.12g%+.12g",
                         descriptor->did,
                         descriptor->name != NULL ? descriptor->name : "unknown",
                         decoded_value,
                         descriptor->unit != NULL ? descriptor->unit : "raw",
                         (unsigned)descriptor->decode_kind,
                         descriptor->scale,
                         descriptor->offset);
                if (s_did_scan.session_file != NULL) {
                    fprintf(s_did_scan.session_file,
                            "[OEM_DID_DECODE]\nDID=0x%04X\nNAME=%s\nVALUE=%.9f\nUNIT=%s\nDECODE_KIND=%u\nSCALE=%.12g\nOFFSET=%.12g\n",
                            descriptor->did,
                            descriptor->name != NULL ? descriptor->name : "unknown",
                            decoded_value,
                            descriptor->unit != NULL ? descriptor->unit : "raw",
                            (unsigned)descriptor->decode_kind,
                            descriptor->scale,
                            descriptor->offset);
                    fflush(s_did_scan.session_file);
                }
            } else {
                ESP_LOGI(TAG,
                         "[OEM_DID_DECODE] DID=0x%04X NAME=\"%s\" STATUS=RAW_ONLY DATA_LENGTH=%u DATA=[%s] UNIT=%s",
                         descriptor->did,
                         descriptor->name != NULL ? descriptor->name : "unknown",
                         (unsigned)event->data_length,
                         response_hex,
                         descriptor->unit != NULL ? descriptor->unit : "raw");
                if (s_did_scan.session_file != NULL) {
                    fprintf(s_did_scan.session_file,
                            "[OEM_DID_DECODE]\nDID=0x%04X\nNAME=%s\nSTATUS=RAW_ONLY\nDATA_LENGTH=%u\nDATA=[%s]\nUNIT=%s\n",
                            descriptor->did,
                            descriptor->name != NULL ? descriptor->name : "unknown",
                            (unsigned)event->data_length,
                            response_hex,
                            descriptor->unit != NULL ? descriptor->unit : "raw");
                    fflush(s_did_scan.session_file);
                }
            }
            s_did_scan.queue_index += 1U;
            s_did_scan.waiting_for_response = false;
            s_did_scan.current_did = 0U;
            return;
        }
    }

    if (event->type == UDS_EVENT_NEGATIVE_RESPONSE) {
        if (event->did == descriptor->did || event->service == descriptor->service) {
            s_did_scan.negative_count += 1U;
            s_did_scan.total_count += 1U;
            char request_hex[64] = {0};
            char negative_hex[32] = {0};
            const uint8_t negative_response[3] = {0x7FU, descriptor->service, event->nrc};
            format_hex_bytes(tx_request, tx_request_length, request_hex, sizeof(request_hex));
            format_hex_bytes(negative_response, sizeof(negative_response), negative_hex, sizeof(negative_hex));
            ESP_LOGI(TAG,
                     "[OEM_DID_SCAN] ECU=%s SERVICE=0x%02X IDENTIFIER=0x%04X TX_UDS=[%s] RX_UDS=[%s] STATUS=NEGATIVE NRC=0x%02X NRC_NAME=%s DURATION_MS=%lld",
                     oem_did_scan_role_name(s_did_scan.role),
                     descriptor->service,
                     descriptor->did,
                     request_hex,
                     negative_hex,
                     event->nrc,
                     nrc_name(event->nrc),
                     (long long)duration_ms);
            obd_client_did_scan_append_record("negative", descriptor->did, tx_request,
                                              tx_request_length, negative_response,
                                              sizeof(negative_response), "NEGATIVE",
                                              event->nrc, false, duration_ms);
            s_did_scan.queue_index += 1U;
            s_did_scan.waiting_for_response = false;
            s_did_scan.current_did = 0U;
            return;
        }
    }

    if (event->type == UDS_EVENT_ERROR && s_did_scan.waiting_for_response) {
        s_did_scan.timeout_count += 1U;
        s_did_scan.total_count += 1U;
        char request_hex[64] = {0};
        format_hex_bytes(tx_request, tx_request_length, request_hex, sizeof(request_hex));
        ESP_LOGI(TAG,
                 "[OEM_DID_SCAN] ECU=%s SERVICE=0x%02X IDENTIFIER=0x%04X TX_UDS=[%s] RX_UDS=[] STATUS=TIMEOUT DURATION_MS=%lld",
                 oem_did_scan_role_name(s_did_scan.role), descriptor->service,
                 descriptor->did, request_hex, (long long)duration_ms);
        obd_client_did_scan_append_record("timeout", descriptor->did, tx_request,
                                          tx_request_length, NULL, 0U,
                                          "TIMEOUT", 0U, false, duration_ms);
        s_did_scan.queue_index += 1U;
        s_did_scan.waiting_for_response = false;
        s_did_scan.current_did = 0U;
    }
}

static void obd_client_did_scan_reset_state(void)
{
    obd_client_did_scan_close_session();
    memset(&s_did_scan, 0, sizeof(s_did_scan));
    s_did_scan.state = OBD_DID_SCAN_STATE_IDLE;
    s_did_scan.active = false;
    s_did_scan.current_did = 0U;
    s_did_scan.queue_count = 0U;
    s_did_scan.queue_index = 0U;
    s_did_scan.total_count = 0U;
}

esp_err_t obd_client_did_scan_start(uint8_t ecu_index, const char *profile_id)
{
    if (ecu_index >= OBD_ECU_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_started || !s_polling_enabled ||
        s_can_discovery.phase != CAN_DISCOVERY_LOCKED) {
        ESP_LOGW(TAG, "[OEM_DID_SCAN] Refused: CAN link is not verified and locked");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_uds_channel_used[ecu_index]) {
        return ESP_ERR_NOT_FOUND;
    }
    if (s_did_scan.state == OBD_DID_SCAN_STATE_RUNNING || s_did_scan.active ||
        s_vehicle_identification_pending || obd_client_any_uds_transaction_active()) {
        return ESP_ERR_INVALID_STATE;
    }

    const isotp_config_t *route = &s_uds[ecu_index].transport.config;
    const diag_ecu_role_t observed_role = diag_ecu_role_for_rx_id(route->rx_id,
                                                                   route->extended_id);
    if ((observed_role != DIAG_ECU_ROLE_ENGINE &&
         observed_role != DIAG_ECU_ROLE_TRANSMISSION) ||
        !diag_ecu_route_is_allowed(route->tx_id, route->rx_id,
                                   route->extended_id, route->extended_id,
                                   observed_role)) {
        ESP_LOGW(TAG,
                 "[OEM_DID_SCAN] Refused: ECU route TX=0x%lX RX=0x%lX is not enabled for read-only diagnostics",
                 (unsigned long)route->tx_id, (unsigned long)route->rx_id);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (profile_id != NULL && profile_id[0] != '\0' &&
        !diag_ecu_profile_is_known(profile_id)) {
        ESP_LOGW(TAG,
                 "[OEM_DID_SCAN] Profile '%s' is not catalogued; continuing with source-backed manual candidates",
                 profile_id);
        profile_id = NULL;
    }

    memset(&s_did_scan, 0, sizeof(s_did_scan));
    s_did_scan.active = true;
    s_did_scan.state = OBD_DID_SCAN_STATE_RUNNING;
    s_did_scan.ecu_index = ecu_index;
    s_did_scan.role = observed_role;
    s_did_scan.allow_experimental = false;
    s_did_scan.current_did = 0U;
    s_did_scan.session_started_us = esp_timer_get_time();

    size_t catalog_count = 0U;
    const uds_did_descriptor_t *catalog = uds_did_catalog_table(&catalog_count);
    for (size_t i = 0; i < catalog_count; ++i) {
        if (catalog[i].role != s_did_scan.role) continue;
        /*
         * In broad exploratory mode, also try identity catalog entries not
         * already queried by V. Keep the V list itself out of D to avoid
         * duplicate traffic and duplicate log records.
         */
        const bool remaining_identity_did =
            profile_id == NULL && s_did_scan.role == DIAG_ECU_ROLE_ENGINE &&
            catalog[i].profile_id != NULL &&
            strcmp(catalog[i].profile_id, "vag-uds-identity") == 0 &&
            !obd_client_vehicle_identity_did_is_queried_by_v(catalog[i].did);
        if (!catalog[i].manual_probe_allowed && !remaining_identity_did) continue;
        if (profile_id != NULL && profile_id[0] != '\0' &&
            catalog[i].profile_id != NULL && catalog[i].profile_id[0] != '\0' &&
            strcmp(catalog[i].profile_id, profile_id) != 0) {
            continue;
        }
        if (!s_did_scan.allow_experimental &&
            catalog[i].validation < UDS_DID_VALIDATION_SOURCE_BACKED) {
            continue;
        }
        if (s_did_scan.queue_count >= (sizeof(s_did_scan.queue) / sizeof(s_did_scan.queue[0]))) {
            break;
        }
        s_did_scan.queue[s_did_scan.queue_count++] = &catalog[i];
    }

    if (s_did_scan.queue_count == 0U) {
        s_did_scan.state = OBD_DID_SCAN_STATE_ERROR;
        s_did_scan.active = false;
        return ESP_ERR_NOT_FOUND;
    }

    s_polling_enabled = false;
    obd_client_did_scan_prepare_session_file(profile_id);
    ESP_LOGI(TAG, "[OEM_DID_SCAN] Started for ECU=%s queue_count=%lu state=%s",
             oem_did_scan_role_name(s_did_scan.role),
             (unsigned long)s_did_scan.queue_count,
             obd_did_scan_state_name(s_did_scan.state));
    return ESP_OK;
}

/*
 * D-command exploratory mode deliberately ignores profile_id. During vehicle
 * characterization we want to test candidates collected from every catalog
 * profile against the attached vehicle, rather than prematurely filtering by
 * the currently inferred ECU profile.
 */
esp_err_t obd_client_did_scan_powertrain_start(void)
{
    if (s_did_scan_powertrain_active) {
        return ESP_ERR_INVALID_STATE;
    }

    s_did_scan_powertrain_active = true;
    s_did_scan_tcu_pending = true;
    const esp_err_t err = obd_client_did_scan_start(0U, NULL);
    if (err != ESP_OK) {
        s_did_scan_powertrain_active = false;
        s_did_scan_tcu_pending = false;
        return err;
    }

    ESP_LOGI(TAG,
             "[OEM_DID_SCAN] Broad powertrain sweep queued: engine candidates plus identity DIDs not already queried by V, followed by TCU candidates when the configured route is available; profile filtering disabled");
    return ESP_OK;
}

static void obd_client_did_scan_powertrain_continue(void)
{
    if (!s_did_scan_powertrain_active) {
        return;
    }

    if (!s_did_scan_tcu_pending) {
        s_did_scan_powertrain_active = false;
        ESP_LOGI(TAG, "[OEM_DID_SCAN] Broad powertrain sweep completed");
        return;
    }

    /*
     * The only currently enabled TCU route is standard 11-bit 0x7E1 -> 0x7E9.
     * Do not guess a 29-bit TCU address: the present 29-bit channel discovery
     * classifies 18DAF1xx responders as engine ECUs.
     */
    s_did_scan_tcu_pending = false;
    if (s_can_config.extended_ids || !s_uds_channel_used[1U]) {
        ESP_LOGW(TAG,
                 "[OEM_DID_SCAN] TCU sweep skipped: no configured 11-bit 0x7E1 -> 0x7E9 route; no 29-bit TCU address will be guessed");
        s_did_scan_powertrain_active = false;
        return;
    }

    const isotp_config_t *route = &s_uds[1U].transport.config;
    if (route->extended_id || route->tx_id != 0x7E1U || route->rx_id != 0x7E9U) {
        ESP_LOGW(TAG,
                 "[OEM_DID_SCAN] TCU sweep skipped: ECU channel 1 is not the configured 0x7E1 -> 0x7E9 route");
        s_did_scan_powertrain_active = false;
        return;
    }

    const esp_err_t err = obd_client_did_scan_start(1U, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "[OEM_DID_SCAN] TCU sweep could not start: %s",
                 esp_err_to_name(err));
        s_did_scan_powertrain_active = false;
        return;
    }

    ESP_LOGI(TAG,
             "[OEM_DID_SCAN] Engine phase finished; starting broad TCU candidate scan on TX=0x7E1 RX=0x7E9");
}

void obd_client_did_scan_stop(void)
{
    s_did_scan_powertrain_active = false;
    s_did_scan_tcu_pending = false;
    if (!s_did_scan.active && s_did_scan.state == OBD_DID_SCAN_STATE_IDLE) {
        return;
    }
    log_oem_did_scan_summary();
    s_did_scan.state = OBD_DID_SCAN_STATE_COMPLETED;
    s_polling_enabled = true;
    obd_client_did_scan_reset_state();
}

void obd_client_did_scan_poll(void)
{
    if (!s_did_scan.active || s_did_scan.state != OBD_DID_SCAN_STATE_RUNNING ||
        s_did_scan.ecu_index >= OBD_ECU_COUNT) {
        return;
    }
    if (s_did_scan.waiting_for_response) {
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= s_did_scan.deadline_us) {
            const uds_did_descriptor_t *descriptor = s_did_scan.queue[s_did_scan.queue_index];
            if (descriptor != NULL) {
                const int64_t duration_ms = (now_us - s_did_scan.request_sent_us) / 1000LL;
                uint8_t tx_request[4] = {0};
                const size_t tx_request_length = obd_client_did_build_request(descriptor, tx_request);
                char request_hex[64] = {0};
                format_hex_bytes(tx_request, tx_request_length, request_hex, sizeof(request_hex));
                ESP_LOGI(TAG,
                         "[OEM_DID_SCAN] ECU=%s SERVICE=0x%02X IDENTIFIER=0x%04X TX_UDS=[%s] RX_UDS=[] STATUS=TIMEOUT DURATION_MS=%lld",
                         oem_did_scan_role_name(s_did_scan.role), descriptor->service,
                         descriptor->did, request_hex, (long long)duration_ms);
                s_did_scan.timeout_count += 1U;
                s_did_scan.total_count += 1U;
                obd_client_did_scan_append_record("timeout", descriptor->did, tx_request,
                                                  tx_request_length, NULL, 0U,
                                                  "TIMEOUT", 0U, false, duration_ms);
            }
            s_did_scan.waiting_for_response = false;
            s_did_scan.current_did = 0U;
            s_did_scan.queue_index += 1U;
        }
        return;
    }

    if (s_did_scan.queue_index >= s_did_scan.queue_count) {
        log_oem_did_scan_summary();
        s_did_scan.state = OBD_DID_SCAN_STATE_COMPLETED;
        s_polling_enabled = true;
        obd_client_did_scan_close_session();
        obd_client_did_scan_reset_state();
        obd_client_did_scan_powertrain_continue();
        return;
    }
    const uds_did_descriptor_t *descriptor = s_did_scan.queue[s_did_scan.queue_index];
    if (descriptor == NULL) {
        s_did_scan.queue_index += 1U;
        return;
    }

    s_did_scan.current_did = descriptor->did;
    s_did_scan.request_sent_us = esp_timer_get_time();
    s_did_scan.deadline_us = s_did_scan.request_sent_us + 2000000LL;
    s_did_scan.waiting_for_response = true;
    uint8_t request[4] = {0};
    const size_t request_length = obd_client_did_build_request(descriptor, request);
    char request_hex[64] = {0};
    format_hex_bytes(request, request_length, request_hex, sizeof(request_hex));
    esp_err_t err = request_length == 0U ? ESP_ERR_INVALID_ARG :
        obd_client_did_send_descriptor(s_did_scan.ecu_index, descriptor,
                                       (uint64_t)s_did_scan.request_sent_us);
    if (err != ESP_OK) {
        s_did_scan.waiting_for_response = false;
        s_did_scan.current_did = 0U;
        ESP_LOGW(TAG, "[OEM_DID_SCAN] SERVICE=0x%02X IDENTIFIER=0x%04X request failed: %s",
                 descriptor->service, descriptor->did, esp_err_to_name(err));
        obd_client_did_scan_append_record("request_failed", descriptor->did, request,
                                          request_length, NULL, 0U,
                                          "ERROR", 0U, false, 0LL);
        s_did_scan.queue_index += 1U;
        return;
    }
    ESP_LOGI(TAG,
             "[OEM_DID_SCAN] ECU=%s SERVICE=0x%02X IDENTIFIER=0x%04X TX_UDS=[%s] STATUS=REQUEST_SENT DECODER=%s",
             oem_did_scan_role_name(s_did_scan.role), descriptor->service,
             descriptor->did, request_hex,
             descriptor->decoder != NULL ? descriptor->decoder : "raw only");
    if (s_did_scan.session_file != NULL) {
        fprintf(s_did_scan.session_file,
                "[OEM_DID_SCAN]\n"
                "ECU=%s\n"
                "SERVICE=0x%02X\n"
                "IDENTIFIER=0x%04X\n"
                "TX_UDS=[%s]\n"
                "DECODER=%s\n"
                "STATUS=REQUEST_SENT\n\n",
                oem_did_scan_role_name(s_did_scan.role), descriptor->service,
                descriptor->did, request_hex,
                descriptor->decoder != NULL ? descriptor->decoder : "raw only");
        fflush(s_did_scan.session_file);
    }
}

void obd_client_set_pid_polling_enabled(bool enabled)
{
    s_pid_polling_enabled = enabled;
    if (enabled) {
        ESP_LOGI(TAG, "[PID] Polling STARTED");
    } else {
        ESP_LOGI(TAG, "[PID] Polling STOPPED");
    }
}

bool obd_client_is_pid_polling_enabled(void)
{
    return s_pid_polling_enabled;
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
#else
    esp_err_t err = can_get_config(&s_can_config);
    if (err != ESP_OK) return err;
    can_discovery_init(&s_can_discovery, &s_can_config);
    s_can_discovery_enabled = true;
    s_discovery_retry_us = 0;
    err = begin_can_candidate_listen(esp_timer_get_time());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start passive CAN candidate scan: %s", esp_err_to_name(err));
        return err;
    }
#if OBD_ACTIVE_TX_ENABLED
    ESP_LOGW(TAG,
             "Active OBD discovery is explicitly enabled: Listen-Only candidates first; TX only after valid bus traffic");
    onx_display_show_waiting_for_rpm("PASSIVE CAN SCAN");
#else
    ESP_LOGI(TAG, "Passive CAN auto-detection enabled; CAN transmission remains disabled");
    onx_display_show_waiting_for_rpm("PASSIVE CAN SCAN");
#endif
#endif

    parameter_manager_init();
    err = initialize_isotp_channels();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialize ISO-TP endpoints: %s", esp_err_to_name(err));
        return err;
    }

    s_started = true;
    s_pid_polling_enabled = true;
    s_vehicle_identification_started = false;
    s_vehicle_identification_pending = false;
    s_vehicle_identity_queue_index = 0U;
    obd_vehicle_reset_identity_state();
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
    memset(s_pid_status, 0, sizeof(s_pid_status));
    memset(s_pid_source_valid, 0, sizeof(s_pid_source_valid));
    memset(s_pid_source_id, 0, sizeof(s_pid_source_id));
    memset(s_pid_source_extended, 0, sizeof(s_pid_source_extended));
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
    if (OBD_LOOPBACK_SIM_ENABLED) {
        s_can_discovery_enabled = false;
    }
    ESP_LOGI(TAG, "Monitor ready: configured bitrate=%lu kbps CAN IDs=%s; live bus monitoring active",
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

static void did_poll_reset(void)
{
    memset(&s_dp, 0, sizeof(s_dp));
    s_dp.current = -1;
}

static const char *did_poll_phase_name(void)
{
    return s_dp.phase == DID_POLL_PHASE_ACTIVE ? "ACTIVE" : "DISCOVERY";
}

static void did_poll_publish(const did_poll_entry_t *entry, double value,
                             const char *state, int64_t now_us)
{
    onx_display_update_did(entry->descriptor->did, entry->descriptor->name,
                           entry->descriptor->unit, (float)value, state,
                           entry->interval_ms, now_us);
}

/* Builds the profile pool from the VIN/ECU identity and arms the supported-DID rounds. */
static void did_poll_arm_after_identity(void)
{
    did_poll_reset();
    if (!s_uds_channel_used[0]) return;

    const isotp_config_t *route = &s_uds[0].transport.config;
    const diag_ecu_role_t role = diag_ecu_role_for_rx_id(route->rx_id, route->extended_id);
    s_dp.brand = did_profile_brand_from_vin(s_vehicle_identity.vin_known ? s_vehicle_identity.vin : NULL);
    const char *pool[DID_PROFILE_POOL_MAX] = {0};
    const size_t pool_count = did_profile_pool(
        s_dp.brand, obd_vehicle_string_known(s_vehicle_identity.f197) ? s_vehicle_identity.f197 : NULL,
        pool);

    char pool_text[160] = {0};
    size_t used = 0U;
    for (size_t i = 0; i < pool_count && used + 1U < sizeof(pool_text); ++i) {
        used += (size_t)snprintf(pool_text + used, sizeof(pool_text) - used,
                                 i == 0U ? "%s" : ",%s", pool[i]);
    }
    const char *family = did_profile_engine_family(
        obd_vehicle_string_known(s_vehicle_identity.f197) ? s_vehicle_identity.f197 : NULL);
    ESP_LOGI(TAG,
             "[PROFILE] BRAND=%s VIN=%s MODEL_YEAR=%s F197=\"%s\" ENGINE_FAMILY=%s ECU_PROFILE=%s MATCH=%s ROLE=%s POOL_PROFILES=%u",
             did_profile_brand_name(s_dp.brand), s_vehicle_identity.vin,
             s_vehicle_identity.model_year, s_vehicle_identity.f197,
             family != NULL ? family : "UNKNOWN",
             s_vehicle_identity.ecu_profile,
             s_vehicle_identity.match_level == OBD_VEHICLE_MATCH_EXACT ? "EXACT" :
             (s_vehicle_identity.match_level == OBD_VEHICLE_MATCH_PARTIAL ? "PARTIAL" : "UNKNOWN"),
             oem_did_scan_role_name(role), (unsigned)pool_count);
    ESP_LOGD(TAG, "[PROFILE] pool=[%s]", pool_text);

    if (role != DIAG_ECU_ROLE_ENGINE || pool_count == 0U) {
        ESP_LOGW(TAG, "[PROFILE] No DID profile for this ECU; supported-DID polling disabled (PIDs unaffected)");
        return;
    }
    const size_t count = did_poll_set_build(&s_dp.set, role, pool, pool_count);
    if (count == 0U) return;
    s_dp.phase = DID_POLL_PHASE_DISCOVERY;
    ESP_LOGI(TAG, "[DID_POLL] Round 1 armed: %u candidate DIDs; NRC 0x31 DIDs will be excluded",
             (unsigned)count);
}

#if OBD_ACTIVE_TX_ENABLED && !OBD_LOOPBACK_SIM_ENABLED
static bool did_poll_try_send(int64_t now_us)
{
    if (s_dp.phase == DID_POLL_PHASE_IDLE || s_dp.waiting || s_did_scan.active ||
        s_vehicle_identification_pending || !s_discovery_complete ||
        !s_uds_channel_used[0] || s_can_discovery.phase != CAN_DISCOVERY_LOCKED) {
        return false;
    }
    int index = -1;
    if (s_dp.phase == DID_POLL_PHASE_DISCOVERY) {
        index = did_poll_next_untested(&s_dp.set);
        if (index < 0) {
            s_dp.phase = DID_POLL_PHASE_ACTIVE;
            const size_t supported = did_poll_count(&s_dp.set, DID_POLL_SUPPORTED);
            const size_t not_ready = did_poll_count(&s_dp.set, DID_POLL_NOT_READY);
            ESP_LOGI(TAG,
                     "[DID_POLL] Round 1 complete: candidates=%u supported=%u not_ready=%u excluded=%u; active poll set=%u",
                     (unsigned)s_dp.set.count, (unsigned)supported, (unsigned)not_ready,
                     (unsigned)did_poll_count(&s_dp.set, DID_POLL_EXCLUDED),
                     (unsigned)(supported + not_ready));
        }
    }
    if (s_dp.phase == DID_POLL_PHASE_ACTIVE) index = did_poll_next_due(&s_dp.set, now_us);
    if (index < 0) return false;

    const esp_err_t err = obd_client_did_send_descriptor(
        0U, s_dp.set.entries[index].descriptor, (uint64_t)now_us);
    if (err == ESP_ERR_INVALID_STATE) return false;
    if (err != ESP_OK) {
        did_poll_record_timeout(&s_dp.set, (size_t)index, now_us);
        return false;
    }
    s_dp.current = index;
    s_dp.waiting = true;
    s_dp.last_was_did = true;
    s_dp.sent_us = now_us;
    return true;
}
#endif

static void did_poll_handle_event(uint8_t ecu_index, const uds_event_t *event)
{
    if (!s_dp.waiting || ecu_index != 0U || event == NULL || s_dp.current < 0) return;
    did_poll_entry_t *entry = &s_dp.set.entries[s_dp.current];
    const uds_did_descriptor_t *d = entry->descriptor;
    const int64_t now_us = esp_timer_get_time();
    const long long duration_ms = (long long)((now_us - s_dp.sent_us) / 1000LL);
    const char *profile = d->profile_id != NULL ? d->profile_id : "?";

    switch (event->type) {
    case UDS_EVENT_RESPONSE: {
        if (event->service != d->service || event->did != d->did) return;
        char raw_hex[256] = {0};
        format_hex_bytes(event->data, event->data_length, raw_hex, sizeof(raw_hex));
        double value = 0.0;
        const bool decoded = uds_did_catalog_decode(d, event->data, event->data_length, &value);
        did_poll_record_positive(&s_dp.set, (size_t)s_dp.current, now_us);
        ++s_dp.positive_count;
        if (decoded) {
            ESP_LOGI(TAG,
                     "[DID_POLL] PHASE=%s PROFILE=%s SERVICE=0x%02X DID=0x%04X STATUS=POSITIVE NAME=\"%s\" VALUE=%.3f UNIT=%s RAW_LEN=%u RAW=[%s] DURATION_MS=%lld",
                     did_poll_phase_name(), profile, d->service, d->did,
                     d->name != NULL ? d->name : "unknown", value,
                     d->unit != NULL ? d->unit : "raw",
                     (unsigned)event->data_length, raw_hex, duration_ms);
            did_poll_publish(entry, value, NULL, now_us);
        } else {
            ESP_LOGI(TAG,
                     "[DID_POLL] PHASE=%s PROFILE=%s SERVICE=0x%02X DID=0x%04X STATUS=POSITIVE_UNDECODED NAME=\"%s\" RAW_LEN=%u RAW=[%s] DURATION_MS=%lld",
                     did_poll_phase_name(), profile, d->service, d->did,
                     d->name != NULL ? d->name : "unknown",
                     (unsigned)event->data_length, raw_hex, duration_ms);
            did_poll_publish(entry, 0.0, "NO DECODE", now_us);
        }
        s_dp.waiting = false;
        break;
    }
    case UDS_EVENT_RESPONSE_PENDING:
        if (entry->state != DID_POLL_NOT_READY) {
            ESP_LOGI(TAG,
                     "[DID_POLL] PHASE=%s PROFILE=%s SERVICE=0x%02X DID=0x%04X STATUS=PENDING (supported, data not ready) NAME=\"%s\"",
                     did_poll_phase_name(), profile, d->service, d->did,
                     d->name != NULL ? d->name : "unknown");
        }
        did_poll_record_pending(&s_dp.set, (size_t)s_dp.current, now_us);
        did_poll_publish(entry, 0.0, "NOT READY", now_us);
        break; /* the final response/timeout is still to come */
    case UDS_EVENT_NEGATIVE_RESPONSE:
        if (event->did != d->did && event->service != d->service) return;
        did_poll_record_negative(&s_dp.set, (size_t)s_dp.current, event->nrc, now_us);
        ESP_LOGI(TAG,
                 "[DID_POLL] PHASE=%s PROFILE=%s SERVICE=0x%02X DID=0x%04X STATUS=NEGATIVE NRC=0x%02X NRC_NAME=%s ACTION=%s",
                 did_poll_phase_name(), profile, d->service, d->did, event->nrc,
                 nrc_name(event->nrc),
                 entry->state == DID_POLL_EXCLUDED ? "EXCLUDED" : "SUPPORTED_NOT_READY");
        if (entry->state == DID_POLL_NOT_READY) did_poll_publish(entry, 0.0, "NOT READY", now_us);
        s_dp.waiting = false;
        break;
    case UDS_EVENT_ERROR:
        did_poll_record_timeout(&s_dp.set, (size_t)s_dp.current, now_us);
        ESP_LOGI(TAG,
                 "[DID_POLL] PHASE=%s PROFILE=%s SERVICE=0x%02X DID=0x%04X STATUS=TIMEOUT ACTION=%s",
                 did_poll_phase_name(), profile, d->service, d->did,
                 entry->state == DID_POLL_EXCLUDED ? "EXCLUDED" : "RETRY");
        if (entry->state == DID_POLL_NOT_READY) did_poll_publish(entry, 0.0, "NOT READY", now_us);
        s_dp.waiting = false;
        break;
    default:
        break;
    }
}

static void did_poll_watchdog(int64_t now_us)
{
    if (s_dp.waiting && now_us - s_dp.sent_us > 8000000LL && s_dp.current >= 0) {
        did_poll_record_timeout(&s_dp.set, (size_t)s_dp.current, now_us);
        s_dp.waiting = false;
    }
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
        if (!s_uds_channel_used[i]) continue;
        const uds_event_type_t uds_result = uds_client_tick(
            &s_uds[i], (uint64_t)now_us, &s_uds_event);
        if (uds_result != UDS_EVENT_NONE) {
            log_uds_event(i, &s_uds_event);
            if (s_did_scan.active && i == s_did_scan.ecu_index) {
                obd_client_did_scan_handle_event(i, &s_uds_event);
            }
            did_poll_handle_event(i, &s_uds_event);
        }
    }
    if (s_did_scan.active) {
        obd_client_did_scan_poll();
    }
    did_poll_watchdog(now_us);
    parameter_manager_mark_stale(now_us);
    onx_display_poll(now_us);
    poll_rx_overflow_flags(now_us);
    log_controller_diagnostics(now_us);
    (void)expire_obd_request(now_us);
    poll_can_discovery(now_us);
    vehicle_identify_auto();
    if (s_vehicle_identification_started && s_vehicle_identification_pending &&
        !s_waiting_for_response && !s_did_scan.active) {
        queue_vehicle_identity_dids();
    }
#if !OBD_SCHEDULER_ENABLED
    return;
#else
    bool uds_transaction_active = false;
    for (uint8_t i = 0; i < OBD_ECU_COUNT; ++i) {
        uds_transaction_active |= uds_client_transaction_active(&s_uds[i]);
    }
    if (s_did_scan.active || !s_polling_enabled || !obd_client_is_pid_polling_enabled() ||
        s_waiting_for_response || uds_transaction_active || now_us < s_next_request_us) return;

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
        !s_polling_enabled || !obd_client_is_pid_polling_enabled()) {
        return;
    }

    const uint8_t request_pid = select_next_pid(now_us);
#if OBD_ACTIVE_TX_ENABLED && !OBD_LOOPBACK_SIM_ENABLED
    /* Alternate DID and PID transactions; a DID also takes the slot when no PID is due. */
    if (s_discovery_complete && (!s_dp.last_was_did || request_pid == 0xFFU) &&
        did_poll_try_send(now_us)) {
        s_next_request_us = now_us + OBD_REQUEST_PERIOD_US;
        return;
    }
#endif
    if (request_pid == 0xFFU) return;
    s_dp.last_was_did = false;
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
        s_pid_status[request_pid].last_request_us = now_us;
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
        record_pid_error(request_pid, OBD_PID_ERROR_TX);
        onx_display_update_parameter(request_pid, 0.0f, PARAM_VALIDITY_INVALID,
                                     now_us);
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
