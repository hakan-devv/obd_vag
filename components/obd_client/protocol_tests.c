#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "can_discovery.h"
#include "isotp.h"
#include "obd_client_test.h"
#include "obd_client.h"
#include "parameter_manager.h"
#include "pid_scheduler.h"
#include "swipe_gesture.h"
#include "uds_client.h"

static const char *TAG = "protocol_tests";

typedef struct {
    can_frame_t last_frame;
    unsigned call_count;
    esp_err_t result;
} fake_can_t;

static esp_err_t capture_frame(const can_frame_t *frame, void *context)
{
    fake_can_t *fake = context;
    fake->last_frame = *frame;
    ++fake->call_count;
    return fake->result;
}

static isotp_config_t standard_isotp_config(void)
{
    return (isotp_config_t) {
        .tx_id = 0x7E0U,
        .rx_id = 0x7E8U,
        .extended_id = false,
        .padding = 0x00U,
        .rx_block_size = 0U,
        .rx_st_min = 0U,
    };
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        ESP_LOGE(TAG, "%s:%d failed: %s", __func__, __LINE__, #condition); \
        return false; \
    } \
} while (0)

#define RUN_TEST(test_fn) do { \
    ++total; \
    if ((test_fn)()) { \
        ESP_LOGI(TAG, "PASS %s", #test_fn); \
    } else { \
        ++failed; \
    } \
} while (0)

static bool test_isotp_single_frame_send(void)
{
    fake_can_t fake = {0};
    isotp_context_t context;
    const isotp_config_t config = standard_isotp_config();
    const uint8_t payload[] = {0x01U, 0x0CU, 0x00U};

    CHECK(isotp_init(&context, &config, capture_frame, &fake) == ESP_OK);
    CHECK(isotp_send(&context, payload, sizeof(payload), 0U) == ESP_OK);
    CHECK(fake.call_count == 1U);
    CHECK(fake.last_frame.data[0] == 0x03U);
    CHECK(memcmp(&fake.last_frame.data[1], payload, sizeof(payload)) == 0);
    CHECK(fake.last_frame.data[7] == 0x00U);
    return true;
}

static bool test_isotp_single_frame_receive(void)
{
    fake_can_t fake = {0};
    static isotp_context_t context;
    const isotp_config_t config = standard_isotp_config();
    const can_frame_t frame = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x03U, 0x41U, 0x0CU, 0x1FU, 0U, 0U, 0U, 0U},
    };
    static isotp_event_t event;

    memset(&event, 0, sizeof(event));
    CHECK(isotp_init(&context, &config, capture_frame, &fake) == ESP_OK);
    CHECK(isotp_on_frame(&context, &frame, 10U, &event) == ISOTP_EVENT_PDU_RECEIVED);
    CHECK(event.length == 3U);
    CHECK(memcmp(event.data, &frame.data[1], event.length) == 0);
    return true;
}

static bool test_isotp_multiframe_receive_and_flow_control(void)
{
    fake_can_t fake = {0};
    static isotp_context_t context;
    const isotp_config_t config = standard_isotp_config();
    const can_frame_t first = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x10U, 0x08U, 1U, 2U, 3U, 4U, 5U, 6U},
    };
    const can_frame_t consecutive = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x21U, 7U, 8U, 0U, 0U, 0U, 0U, 0U},
    };
    static isotp_event_t event;

    memset(&event, 0, sizeof(event));
    CHECK(isotp_init(&context, &config, capture_frame, &fake) == ESP_OK);
    CHECK(isotp_on_frame(&context, &first, 100U, &event) == ISOTP_EVENT_NONE);
    CHECK(fake.last_frame.data[0] == 0x30U);
    CHECK(fake.last_frame.data[1] == 0U);
    CHECK(isotp_on_frame(&context, &consecutive, 200U, &event) ==
          ISOTP_EVENT_PDU_RECEIVED);
    CHECK(event.length == 8U);
    for (uint8_t i = 0; i < 8U; ++i) CHECK(event.data[i] == i + 1U);
    return true;
}

static bool test_isotp_sequence_mismatch(void)
{
    fake_can_t fake = {0};
    static isotp_context_t context;
    const isotp_config_t config = standard_isotp_config();
    const can_frame_t first = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x10U, 0x08U, 1U, 2U, 3U, 4U, 5U, 6U},
    };
    const can_frame_t wrong_sequence = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x22U, 7U, 8U, 0U, 0U, 0U, 0U, 0U},
    };
    static isotp_event_t event;

    memset(&event, 0, sizeof(event));
    CHECK(isotp_init(&context, &config, capture_frame, &fake) == ESP_OK);
    CHECK(isotp_on_frame(&context, &first, 100U, &event) == ISOTP_EVENT_NONE);
    CHECK(isotp_on_frame(&context, &wrong_sequence, 200U, &event) ==
          ISOTP_EVENT_ERROR);
    CHECK(event.error == ISOTP_ERROR_SEQUENCE);
    return true;
}

static bool test_isotp_flow_control_direction(void)
{
    fake_can_t fake = {0};
    static isotp_context_t context;
    const isotp_config_t config = standard_isotp_config();
    const uint8_t payload[] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
    const can_frame_t flow_control = {
        .id = 0x7E8U, .dlc = 3U, .extended = false, .rtr = false,
        .data = {0x30U, 0U, 0U},
    };
    static isotp_event_t event;

    memset(&event, 0, sizeof(event));
    CHECK(isotp_init(&context, &config, capture_frame, &fake) == ESP_OK);
    CHECK(isotp_send(&context, payload, sizeof(payload), 100U) == ESP_OK);
    CHECK(fake.last_frame.data[0] == 0x10U);
    CHECK(isotp_on_frame(&context, &flow_control, 200U, &event) == ISOTP_EVENT_NONE);
    CHECK(isotp_tick(&context, 200U, &event) == ISOTP_EVENT_TX_COMPLETE);
    CHECK(fake.last_frame.data[0] == 0x21U);
    CHECK(fake.last_frame.data[1] == 7U);
    CHECK(fake.last_frame.data[2] == 8U);
    return true;
}

static bool test_isotp_receive_timeout(void)
{
    fake_can_t fake = {0};
    static isotp_context_t context;
    const isotp_config_t config = standard_isotp_config();
    const can_frame_t first = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x10U, 0x08U, 1U, 2U, 3U, 4U, 5U, 6U},
    };
    static isotp_event_t event;

    memset(&event, 0, sizeof(event));
    CHECK(isotp_init(&context, &config, capture_frame, &fake) == ESP_OK);
    CHECK(isotp_on_frame(&context, &first, 100U, &event) == ISOTP_EVENT_NONE);
    CHECK(isotp_tick(&context, 1000100U, &event) == ISOTP_EVENT_ERROR);
    CHECK(event.error == ISOTP_ERROR_RX_TIMEOUT);
    return true;
}

static bool test_pid_bitmap_discovery(void)
{
    const uint8_t bitmap[4] = {0x08U, 0x10U, 0x00U, 0x01U};

    parameter_manager_init();
    CHECK(parameter_manager_record_supported_page(0x7E8U, false, 0x00U,
                                                   bitmap) == ESP_OK);
    CHECK(parameter_manager_pid_support_known(0x7E8U, false, 0x05U));
    CHECK(parameter_manager_pid_supported(0x7E8U, false, 0x05U));
    CHECK(parameter_manager_pid_supported(0x7E8U, false, 0x0CU));
    CHECK(parameter_manager_page_has_continuation(bitmap));
    CHECK(!parameter_manager_pid_supported(0x7E8U, false, 0x0DU));
    const uint8_t next_page[4] = {0x80U, 0U, 0U, 0U};
    CHECK(parameter_manager_record_supported_page(0x7E8U, false, 0x20U,
                                                   next_page) == ESP_OK);
    CHECK(parameter_manager_pid_supported(0x7E8U, false, 0x21U));
    return true;
}

static bool test_standard_pid_decoders(void)
{
    parameter_value_t value = {0};
    const uint8_t rpm_data[2] = {0x1FU, 0x40U};
    const uint8_t coolant_data[1] = {0x5AU};

    parameter_manager_init();
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x0CU, rpm_data,
        sizeof(rpm_data), 100U, &value) == ESP_OK);
    CHECK(value.value > 1999.99f && value.value < 2000.01f);
    CHECK(value.validity == PARAM_VALIDITY_VALID);
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x05U, coolant_data,
        sizeof(coolant_data), 200U, &value) == ESP_OK);
    CHECK(value.value > 49.99f && value.value < 50.01f);
    return true;
}

static bool test_pid_decoder_rejects_bad_data_and_unknown_pid(void)
{
    parameter_value_t value = {0};
    const uint8_t short_rpm[1] = {0x1FU};

    parameter_manager_init();
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x0CU, short_rpm,
        sizeof(short_rpm), 100U, &value) == ESP_ERR_INVALID_SIZE);
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0xFEU, short_rpm,
        sizeof(short_rpm), 100U, &value) == ESP_ERR_NOT_SUPPORTED);
    return true;
}

static bool test_pid_stale_transition(void)
{
    parameter_value_t value = {0};
    const uint8_t coolant_data[1] = {0x5AU};

    parameter_manager_init();
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x05U, coolant_data,
        sizeof(coolant_data), 10U, &value) == ESP_OK);
    parameter_manager_mark_stale(3000009LL);
    CHECK(parameter_manager_get_value(0x7E8U, false, 0x05U, &value));
    CHECK(value.validity == PARAM_VALIDITY_VALID);
    parameter_manager_mark_stale(3000010LL);
    CHECK(parameter_manager_get_value(0x7E8U, false, 0x05U, &value));
    CHECK(value.validity == PARAM_VALIDITY_STALE);
    return true;
}

static bool test_pid_timeout_status_and_recovery(void)
{
    parameter_value_t value = {0};
    const uint8_t coolant_data[1] = {0x5AU};

    parameter_manager_init();
    parameter_manager_record_timeout(0x05U, 100U);
    CHECK(parameter_manager_get_latest_value(0x05U, &value));
    CHECK(value.validity == PARAM_VALIDITY_TIMEOUT);
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x05U, coolant_data,
        sizeof(coolant_data), 200U, &value) == ESP_OK);
    CHECK(parameter_manager_get_latest_value(0x05U, &value));
    CHECK(value.validity == PARAM_VALIDITY_VALID);
    CHECK(value.value == 50.0f);
    return true;
}

static bool test_obd_negative_response_and_missing_response_timeout(void)
{
    static const uint8_t mode01_negative[] = {0x7FU, 0x01U, 0x11U};
    static const uint8_t unrelated_negative[] = {0x7FU, 0x22U, 0x31U};
    obd_pid_status_t status = {0};

    CHECK(obd_client_test_process_response(0x0CU, mode01_negative,
        sizeof(mode01_negative)));
    CHECK(obd_client_get_pid_status(0x0CU, &status));
    CHECK(status.last_error == OBD_PID_ERROR_NEGATIVE_RESPONSE);
    CHECK(status.error_count == 1U);

    CHECK(!obd_client_test_process_response(0x0CU, unrelated_negative,
        sizeof(unrelated_negative)));
    CHECK(!obd_client_get_pid_status(0x0CU, &status));

    CHECK(!obd_client_test_expire_request(0x0CU, 199999LL));
    CHECK(obd_client_test_expire_request(0x0CU, 200000LL));
    CHECK(obd_client_get_pid_status(0x0CU, &status));
    CHECK(status.last_error == OBD_PID_ERROR_TIMEOUT);
    CHECK(status.error_count == 1U);
    return true;
}

static bool test_extended_standard_pid_decoders(void)
{
    parameter_value_t value = {0};
    const uint8_t fuel_pressure[] = {16U};
    const uint8_t absolute_load[] = {0x01U, 0x00U};
    const uint8_t fuel_rate[] = {0x01U, 0x90U};
    const uint8_t odometer[] = {0U, 0U, 0x27U, 0x10U};

    parameter_manager_init();
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x0AU, fuel_pressure,
        sizeof(fuel_pressure), 100U, &value) == ESP_OK);
    CHECK(value.value == 48.0f);
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x43U, absolute_load,
        sizeof(absolute_load), 200U, &value) == ESP_OK);
    CHECK(value.value > 100.3f && value.value < 100.5f);
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x5EU, fuel_rate,
        sizeof(fuel_rate), 300U, &value) == ESP_OK);
    CHECK(value.value == 20.0f);
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0xA1U, odometer,
        sizeof(odometer), 400U, &value) == ESP_OK);
    CHECK(value.value == 1000.0f);
    CHECK(parameter_manager_get_latest_value(0xA1U, &value));
    CHECK(value.validity == PARAM_VALIDITY_VALID && value.value == 1000.0f);
    return true;
}

static bool test_ecu_simulator_supported_pid_set(void)
{
    static const uint8_t simulator_pids[] = {
        0x04U, 0x05U, 0x0BU, 0x0CU, 0x0DU, 0x0FU, 0x11U,
    };
    static const uint8_t samples[][2] = {
        {0x80U, 0U}, {0x82U, 0U}, {0x22U, 0U}, {0x1CU, 0xC0U},
        {0x29U, 0U}, {0x4BU, 0U}, {0x11U, 0U},
    };
    static const uint8_t lengths[] = {1U, 1U, 1U, 2U, 1U, 1U, 1U};
    uint8_t bitmap[4] = {0U};

    for (size_t i = 0; i < sizeof(simulator_pids); ++i) {
        const uint8_t bit = (uint8_t)(simulator_pids[i] - 1U);
        bitmap[bit / 8U] |= (uint8_t)(0x80U >> (bit % 8U));
    }

    parameter_manager_init();
    CHECK(parameter_manager_record_supported_page(0x7E8U, false, 0x00U,
                                                   bitmap) == ESP_OK);
    for (size_t i = 0; i < sizeof(simulator_pids); ++i) {
        parameter_value_t value = {0};
        CHECK(parameter_manager_pid_supported(0x7E8U, false, simulator_pids[i]));
        CHECK(parameter_manager_decode_obd(0x7E8U, false, simulator_pids[i],
            samples[i], lengths[i], (int64_t)i + 1, &value) == ESP_OK);
        CHECK(value.validity == PARAM_VALIDITY_VALID);
    }
    return true;
}

static bool test_pid_scheduler_fairness(void)
{
    const parameter_descriptor_t catalog[] = {
        {0x0CU, "RPM", "rpm", 2U, PARAM_DECODER_RPM_AB_DIV_4,
         0.0f, 16000.0f, 100U, 0U},
        {0x0DU, "Speed", "km/h", 1U, PARAM_DECODER_SPEED_KMH,
         0.0f, 255.0f, 100U, 1U},
        {0x05U, "Coolant", "C", 1U, PARAM_DECODER_COOLANT_A_MINUS_40,
         -40.0f, 215.0f, 1000U, 3U},
        {0x04U, "Load", "%", 1U, PARAM_DECODER_PERCENT_A,
         0.0f, 100.0f, 500U, 2U},
    };
    bool supported[256] = {false};
    int64_t due[256] = {0};
    supported[0x0CU] = supported[0x0DU] = supported[0x05U] = supported[0x04U] = true;
    int64_t now_us = 0;
    bool saw_coolant = false;
    bool saw_load = false;

    for (unsigned request = 0; request < 24U; ++request) {
        const uint8_t pid = pid_scheduler_select(catalog,
            sizeof(catalog) / sizeof(catalog[0]), supported, due, now_us);
        CHECK(pid != 0xFFU);
        if (pid == 0x05U) saw_coolant = true;
        if (pid == 0x04U) saw_load = true;
        for (size_t i = 0; i < sizeof(catalog) / sizeof(catalog[0]); ++i) {
            if (catalog[i].pid == pid) {
                due[pid] = now_us + (int64_t)catalog[i].update_period_ms * 1000LL;
                break;
            }
        }
        now_us += 50000LL;
    }
    CHECK(saw_coolant);
    CHECK(saw_load);
    return true;
}

static bool test_swipe_gesture_direction_and_threshold(void)
{
    swipe_gesture_t gesture = {0};
    swipe_gesture_begin(&gesture, 190, 140);
    CHECK(swipe_gesture_end(&gesture, 130, 145) == SWIPE_LEFT);

    swipe_gesture_begin(&gesture, 50, 140);
    CHECK(swipe_gesture_end(&gesture, 110, 145) == SWIPE_RIGHT);

    swipe_gesture_begin(&gesture, 100, 100);
    CHECK(swipe_gesture_end(&gesture, 125, 102) == SWIPE_NONE);

    swipe_gesture_begin(&gesture, 100, 80);
    CHECK(swipe_gesture_end(&gesture, 110, 150) == SWIPE_NONE);
    return true;
}

static bool test_can_discovery_candidate_order_and_frame_evidence(void)
{
    can_discovery_t discovery;
    const can_bus_config_t initial = {CAN_BITRATE_500_KBPS, false};
    can_discovery_init(&discovery, &initial);

    const can_bus_config_t expected[] = {
        {CAN_BITRATE_500_KBPS, false},
        {CAN_BITRATE_500_KBPS, true},
        {CAN_BITRATE_250_KBPS, false},
        {CAN_BITRATE_250_KBPS, true},
    };
    for (size_t i = 0; i < CAN_DISCOVERY_CANDIDATE_COUNT; ++i) {
        const can_bus_config_t *candidate = can_discovery_candidate(&discovery);
        CHECK(candidate != NULL);
        CHECK(candidate->bitrate == expected[i].bitrate);
        CHECK(candidate->extended_ids == expected[i].extended_ids);

        const can_frame_t wrong_format = {
            .id = candidate->extended_ids ? 0x7E8U : 0x18DAF110U,
            .dlc = 8U, .extended = !candidate->extended_ids, .rtr = false,
        };
        (void)can_discovery_observe(&discovery, &wrong_format);
        CHECK(!can_discovery_has_evidence(&discovery));
        const can_frame_t valid = {
            .id = candidate->extended_ids ? 0x18DAF110U : 0x7E8U,
            .dlc = 8U, .extended = candidate->extended_ids, .rtr = false,
        };
        CHECK(can_discovery_observe(&discovery, &valid));
        CHECK(can_discovery_observe(&discovery, &valid));
        CHECK(!can_discovery_has_evidence(&discovery));
        CHECK(can_discovery_observe(&discovery, &valid));
        CHECK(can_discovery_has_evidence(&discovery));
        can_discovery_reject_candidate(&discovery);
    }
    CHECK(can_discovery_candidate(&discovery) == NULL);
    return true;
}

static bool test_can_discovery_requires_valid_bitmap_response(void)
{
    can_discovery_t discovery;
    const can_bus_config_t initial = {CAN_BITRATE_500_KBPS, false};
    const can_frame_t observed = {
        .id = 0x123U, .dlc = 8U, .extended = false, .rtr = false,
    };
    const uint8_t valid_bitmap[] = {0x41U, 0x00U, 0x08U, 0x10U, 0x00U, 0x00U};
    const uint8_t negative_response[] = {0x7FU, 0x01U, 0x11U, 0x00U, 0x00U, 0x00U};
    can_frame_t response = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
    };

    can_discovery_init(&discovery, &initial);
    for (unsigned i = 0; i < CAN_DISCOVERY_MIN_VALID_FRAMES; ++i) {
        CHECK(can_discovery_observe(&discovery, &observed));
    }
    CHECK(can_discovery_begin_verification(&discovery) == ESP_OK);
    CHECK(!can_discovery_confirm(&discovery, &response, negative_response,
                                 sizeof(negative_response)));
    CHECK(!can_discovery_confirm(&discovery, &response, valid_bitmap, 5U));
    response.id = 0x123U;
    CHECK(!can_discovery_confirm(&discovery, &response, valid_bitmap,
                                 sizeof(valid_bitmap)));
    response.id = 0x7E9U;
    CHECK(can_discovery_confirm(&discovery, &response, valid_bitmap,
                                sizeof(valid_bitmap)));
    CHECK(discovery.phase == CAN_DISCOVERY_LOCKED);
    return true;
}

static bool test_can_discovery_extended_response_and_retry(void)
{
    can_discovery_t discovery;
    const can_bus_config_t initial = {CAN_BITRATE_250_KBPS, true};
    const can_frame_t observed = {
        .id = 0x18DAF110U, .dlc = 8U, .extended = true, .rtr = false,
    };
    const uint8_t valid_bitmap[] = {0x41U, 0x00U, 0x08U, 0x10U, 0x00U, 0x00U};
    const uint8_t no_payload[] = {0U};
    can_frame_t response = {
        .id = 0x18DAF1A3U, .dlc = 8U, .extended = true, .rtr = false,
    };

    can_discovery_init(&discovery, &initial);
    for (unsigned i = 0; i < CAN_DISCOVERY_MIN_VALID_FRAMES; ++i) {
        CHECK(can_discovery_observe(&discovery, &observed));
    }
    CHECK(can_discovery_begin_verification(&discovery) == ESP_OK);
    CHECK(!can_discovery_confirm(&discovery, &response, no_payload,
                                 sizeof(no_payload)));
    CHECK(can_discovery_confirm(&discovery, &response, valid_bitmap,
                                sizeof(valid_bitmap)));

    can_discovery_init(&discovery, &initial);
    for (size_t i = 0; i < CAN_DISCOVERY_CANDIDATE_COUNT; ++i) {
        CHECK(can_discovery_candidate(&discovery) != NULL);
        can_discovery_reject_candidate(&discovery);
    }
    CHECK(discovery.phase == CAN_DISCOVERY_EXHAUSTED);
    can_discovery_restart(&discovery);
    CHECK(discovery.phase == CAN_DISCOVERY_LISTENING);
    CHECK(can_discovery_candidate(&discovery) != NULL);
    return true;
}

static bool test_uds_read_did_round_trip(void)
{
    fake_can_t fake = {0};
    static uds_client_t client;
    const isotp_config_t config = standard_isotp_config();
    const can_frame_t response = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x06U, 0x62U, 0xF1U, 0x90U, 0x01U, 0x02U, 0x03U, 0U},
    };
    static uds_event_t event;

    memset(&event, 0, sizeof(event));
    CHECK(uds_client_init(&client, &config, capture_frame, &fake) == ESP_OK);
    CHECK(uds_client_read_did(&client, 0xF190U, 100U) == ESP_OK);
    CHECK(fake.last_frame.data[0] == 0x03U);
    CHECK(fake.last_frame.data[1] == 0x22U);
    CHECK(fake.last_frame.data[2] == 0xF1U);
    CHECK(fake.last_frame.data[3] == 0x90U);
    CHECK(uds_client_on_frame(&client, &response, 200U, &event) ==
          UDS_EVENT_RESPONSE);
    CHECK(event.service == 0x22U);
    CHECK(event.did == 0xF190U);
    CHECK(event.data_length == 3U);
    CHECK(event.data[0] == 1U && event.data[1] == 2U && event.data[2] == 3U);
    CHECK(!uds_client_transaction_active(&client));
    return true;
}

static bool test_uds_extended_session_and_pending_timeout(void)
{
    fake_can_t fake = {0};
    static uds_client_t client;
    const isotp_config_t config = standard_isotp_config();
    const can_frame_t session_response = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x02U, 0x50U, 0x03U, 0U, 0U, 0U, 0U, 0U},
    };
    const can_frame_t response_pending = {
        .id = 0x7E8U, .dlc = 8U, .extended = false, .rtr = false,
        .data = {0x03U, 0x7FU, 0x22U, 0x78U, 0U, 0U, 0U, 0U},
    };
    static uds_event_t event;

    memset(&event, 0, sizeof(event));
    CHECK(uds_client_init(&client, &config, capture_frame, &fake) == ESP_OK);
    CHECK(uds_client_start_extended_session(&client, 100U) == ESP_OK);
    CHECK(uds_client_on_frame(&client, &session_response, 200U, &event) ==
          UDS_EVENT_RESPONSE);
    CHECK(client.session == UDS_SESSION_EXTENDED);
    CHECK(uds_client_read_did(&client, 0x1234U, 300U) == ESP_OK);
    CHECK(uds_client_on_frame(&client, &response_pending, 400U, &event) ==
          UDS_EVENT_RESPONSE_PENDING);
    CHECK(uds_client_transaction_active(&client));
    CHECK(uds_client_tick(&client, 5000399U, &event) == UDS_EVENT_NONE);
    CHECK(uds_client_tick(&client, 5000400U, &event) == UDS_EVENT_ERROR);
    CHECK(event.error == UDS_ERROR_TIMEOUT);
    return true;
}

void obd_client_run_protocol_tests(void)
{
    unsigned total = 0U;
    unsigned failed = 0U;

    RUN_TEST(test_isotp_single_frame_send);
    RUN_TEST(test_isotp_single_frame_receive);
    RUN_TEST(test_isotp_multiframe_receive_and_flow_control);
    RUN_TEST(test_isotp_sequence_mismatch);
    RUN_TEST(test_isotp_flow_control_direction);
    RUN_TEST(test_isotp_receive_timeout);
    RUN_TEST(test_pid_bitmap_discovery);
    RUN_TEST(test_standard_pid_decoders);
    RUN_TEST(test_pid_decoder_rejects_bad_data_and_unknown_pid);
    RUN_TEST(test_pid_stale_transition);
    RUN_TEST(test_pid_timeout_status_and_recovery);
    RUN_TEST(test_obd_negative_response_and_missing_response_timeout);
    RUN_TEST(test_extended_standard_pid_decoders);
    RUN_TEST(test_ecu_simulator_supported_pid_set);
    RUN_TEST(test_pid_scheduler_fairness);
    RUN_TEST(test_swipe_gesture_direction_and_threshold);
    RUN_TEST(test_can_discovery_candidate_order_and_frame_evidence);
    RUN_TEST(test_can_discovery_requires_valid_bitmap_response);
    RUN_TEST(test_can_discovery_extended_response_and_retry);
    RUN_TEST(test_uds_read_did_round_trip);
    RUN_TEST(test_uds_extended_session_and_pending_timeout);

    if (failed == 0U) {
        ESP_LOGI(TAG, "Protocol tests passed: %u/%u", total, total);
    } else {
        ESP_LOGE(TAG, "Protocol tests failed: %u/%u", failed, total);
    }
}
