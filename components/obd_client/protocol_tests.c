#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "isotp.h"
#include "parameter_manager.h"
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

static bool test_pid_decoder_rejects_bad_data(void)
{
    parameter_value_t value = {0};
    const uint8_t short_rpm[1] = {0x1FU};
    const uint8_t invalid_coolant[1] = {0xFFU};

    parameter_manager_init();
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x0CU, short_rpm,
        sizeof(short_rpm), 100U, &value) == ESP_ERR_INVALID_SIZE);
    CHECK(parameter_manager_decode_obd(0x7E8U, false, 0x05U, invalid_coolant,
        sizeof(invalid_coolant), 100U, &value) == ESP_ERR_INVALID_RESPONSE);
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
    RUN_TEST(test_pid_decoder_rejects_bad_data);
    RUN_TEST(test_pid_stale_transition);
    RUN_TEST(test_uds_read_did_round_trip);
    RUN_TEST(test_uds_extended_session_and_pending_timeout);

    if (failed == 0U) {
        ESP_LOGI(TAG, "Protocol tests passed: %u/%u", total, total);
    } else {
        ESP_LOGE(TAG, "Protocol tests failed: %u/%u", failed, total);
    }
}
