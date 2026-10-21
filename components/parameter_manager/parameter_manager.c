#include "parameter_manager.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "parameter_manager";

static const parameter_descriptor_t CATALOG[] = {
    {0x04U, "Calculated engine load", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 2U},
    {0x05U, "Engine coolant temperature", "C", 1U, PARAM_DECODER_COOLANT_A_MINUS_40, -40.0f, 215.0f, 1000U, 3U},
    {0x06U, "Short-term fuel trim Bank 1", "%", 1U, PARAM_DECODER_FUEL_TRIM_A, -100.0f, 99.2f, 1000U, 4U},
    {0x07U, "Long-term fuel trim Bank 1", "%", 1U, PARAM_DECODER_FUEL_TRIM_A, -100.0f, 99.2f, 1000U, 4U},
    {0x08U, "Short-term fuel trim Bank 2", "%", 1U, PARAM_DECODER_FUEL_TRIM_A, -100.0f, 99.2f, 1000U, 6U},
    {0x09U, "Long-term fuel trim Bank 2", "%", 1U, PARAM_DECODER_FUEL_TRIM_A, -100.0f, 99.2f, 1000U, 6U},
    {0x0AU, "Fuel pressure", "kPa", 1U, PARAM_DECODER_PRESSURE_A_TIMES_3, 0.0f, 765.0f, 1000U, 6U},
    {0x0BU, "Intake manifold absolute pressure", "kPa", 1U, PARAM_DECODER_MAP_KPA, 0.0f, 255.0f, 500U, 4U},
    {0x0CU, "Engine speed", "rpm", 2U, PARAM_DECODER_RPM_AB_DIV_4, 0.0f, 16383.75f, 100U, 0U},
    {0x0DU, "Vehicle speed", "km/h", 1U, PARAM_DECODER_SPEED_KMH, 0.0f, 255.0f, 100U, 1U},
    {0x0EU, "Ignition timing advance", "deg", 1U, PARAM_DECODER_IGNITION_A_DIV_2_MINUS_64, -64.0f, 63.5f, 500U, 5U},
    {0x0FU, "Intake air temperature", "C", 1U, PARAM_DECODER_TEMPERATURE_A_MINUS_40, -40.0f, 215.0f, 1000U, 5U},
    {0x10U, "Mass air flow", "g/s", 2U, PARAM_DECODER_MAF_AB_DIV_100, 0.0f, 655.35f, 500U, 4U},
    {0x11U, "Throttle position", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 250U, 2U},
    {0x1FU, "Run time since engine start", "s", 2U, PARAM_DECODER_U16_AB, 0.0f, 65535.0f, 5000U, 7U},
    {0x21U, "Distance with MIL on", "km", 2U, PARAM_DECODER_U16_AB, 0.0f, 65535.0f, 5000U, 7U},
    {0x22U, "Fuel rail pressure", "kPa", 2U, PARAM_DECODER_FUEL_RAIL_PRESSURE_AB, 0.0f, 5176.8f, 1000U, 6U},
    {0x23U, "Diesel rail pressure", "kPa", 2U, PARAM_DECODER_PRESSURE_AB_TIMES_10, 0.0f, 655350.0f, 1000U, 6U},
    {0x2CU, "Commanded EGR", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 1000U, 6U},
    {0x2DU, "EGR error", "%", 1U, PARAM_DECODER_EGR_ERROR_A, -100.0f, 99.2f, 1000U, 6U},
    {0x2EU, "Evaporative purge", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 1000U, 7U},
    {0x2FU, "Fuel tank level input", "%", 1U, PARAM_DECODER_FUEL_LEVEL_PERCENT, 0.0f, 100.0f, 5000U, 7U},
    {0x31U, "Distance since DTC clear", "km", 2U, PARAM_DECODER_U16_AB, 0.0f, 65535.0f, 5000U, 7U},
    {0x33U, "Barometric pressure", "kPa", 1U, PARAM_DECODER_MAP_KPA, 0.0f, 255.0f, 5000U, 7U},
    {0x3CU, "Catalyst temperature B1S1", "C", 2U, PARAM_DECODER_CATALYST_TEMP_AB_DIV_10_MINUS_40, -40.0f, 6513.5f, 5000U, 7U},
    {0x3DU, "Catalyst temperature B2S1", "C", 2U, PARAM_DECODER_CATALYST_TEMP_AB_DIV_10_MINUS_40, -40.0f, 6513.5f, 5000U, 7U},
    {0x3EU, "Catalyst temperature B1S2", "C", 2U, PARAM_DECODER_CATALYST_TEMP_AB_DIV_10_MINUS_40, -40.0f, 6513.5f, 5000U, 7U},
    {0x3FU, "Catalyst temperature B2S2", "C", 2U, PARAM_DECODER_CATALYST_TEMP_AB_DIV_10_MINUS_40, -40.0f, 6513.5f, 5000U, 7U},
    {0x42U, "Control module voltage", "V", 2U, PARAM_DECODER_VOLTAGE_AB_DIV_1000, 0.0f, 65.535f, 2000U, 6U},
    {0x43U, "Absolute engine load", "%", 2U, PARAM_DECODER_ABSOLUTE_LOAD_AB, 0.0f, 25700.0f, 500U, 4U},
    {0x44U, "Commanded equivalence ratio", "ratio", 2U, PARAM_DECODER_EQUIVALENCE_RATIO_AB, 0.0f, 2.0f, 1000U, 6U},
    {0x45U, "Relative throttle position", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x46U, "Ambient air temperature", "C", 1U, PARAM_DECODER_TEMPERATURE_A_MINUS_40, -40.0f, 215.0f, 5000U, 7U},
    {0x47U, "Absolute throttle position B", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x48U, "Absolute throttle position C", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x49U, "Accelerator pedal position D", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x4AU, "Accelerator pedal position E", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x4BU, "Accelerator pedal position F", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x4CU, "Commanded throttle actuator", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x4DU, "Time with MIL on", "min", 2U, PARAM_DECODER_U16_AB, 0.0f, 65535.0f, 5000U, 7U},
    {0x4EU, "Time since DTC clear", "min", 2U, PARAM_DECODER_U16_AB, 0.0f, 65535.0f, 5000U, 7U},
    {0x52U, "Ethanol fuel percentage", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 5000U, 7U},
    {0x53U, "Absolute evaporative pressure", "kPa", 1U, PARAM_DECODER_EVAP_PRESSURE_A_DIV_200, 0.0f, 1.275f, 5000U, 7U},
    {0x5AU, "Relative accelerator pedal position", "%", 1U, PARAM_DECODER_PERCENT_A, 0.0f, 100.0f, 500U, 5U},
    {0x5CU, "Engine oil temperature", "C", 1U, PARAM_DECODER_TEMPERATURE_A_MINUS_40, -40.0f, 215.0f, 5000U, 7U},
    {0x5DU, "Fuel injection timing", "deg", 2U, PARAM_DECODER_INJECTION_TIMING_AB, -210.0f, 301.992f, 1000U, 6U},
    {0x5EU, "Engine fuel rate", "L/h", 2U, PARAM_DECODER_FUEL_RATE_AB, 0.0f, 3276.75f, 1000U, 6U},
    {0x62U, "Actual engine percent torque", "%", 1U, PARAM_DECODER_TORQUE_PERCENT_A_MINUS_125, -125.0f, 130.0f, 1000U, 6U},
    {0x63U, "Engine reference torque", "Nm", 2U, PARAM_DECODER_TORQUE_REFERENCE_AB, 0.0f, 65535.0f, 5000U, 7U},
    {0x74U, "Turbocharger RPM", "rpm", 2U, PARAM_DECODER_TURBO_RPM_AB_TIMES_10, 0.0f, 655350.0f, 1000U, 6U},
    {0xA1U, "Odometer", "km", 4U, PARAM_DECODER_ODOMETER_ABCD_DIV_10, 0.0f, 429496729.5f, 5000U, 7U},
};

typedef struct {
    uint32_t id;
    bool extended;
    bool used;
    uint8_t known_pages[8];
    uint8_t supported[32];
    parameter_value_t values[sizeof(CATALOG) / sizeof(CATALOG[0])];
} ecu_parameters_t;

static ecu_parameters_t s_ecus[PARAMETER_MANAGER_MAX_ECUS];
static parameter_value_t s_global_timeouts[sizeof(CATALOG) / sizeof(CATALOG[0])];

static int catalog_index(uint8_t pid);

static int find_ecu(uint32_t ecu_id, bool extended_id, bool create)
{
    int unused = -1;
    for (size_t i = 0; i < PARAMETER_MANAGER_MAX_ECUS; ++i) {
        if (s_ecus[i].used && s_ecus[i].id == ecu_id &&
            s_ecus[i].extended == extended_id) return (int)i;
        if (!s_ecus[i].used && unused < 0) unused = (int)i;
    }
    if (!create || unused < 0) return -1;
    s_ecus[unused].used = true;
    s_ecus[unused].id = ecu_id;
    s_ecus[unused].extended = extended_id;
    return unused;
}

static int catalog_index(uint8_t pid)
{
    for (size_t i = 0; i < sizeof(CATALOG) / sizeof(CATALOG[0]); ++i) {
        if (CATALOG[i].pid == pid) return (int)i;
    }
    return -1;
}

void parameter_manager_init(void)
{
    memset(s_ecus, 0, sizeof(s_ecus));
    memset(s_global_timeouts, 0, sizeof(s_global_timeouts));
}

const parameter_descriptor_t *parameter_manager_catalog(size_t *count)
{
    if (count != NULL) *count = sizeof(CATALOG) / sizeof(CATALOG[0]);
    return CATALOG;
}

esp_err_t parameter_manager_record_supported_page(uint32_t ecu_id, bool extended_id,
                                                  uint8_t base_pid,
                                                  const uint8_t bitmap[4])
{
    if (bitmap == NULL || (base_pid & 0x1FU) != 0U || base_pid > 0xE0U) {
        return ESP_ERR_INVALID_ARG;
    }
    int index = find_ecu(ecu_id, extended_id, true);
    if (index < 0) return ESP_ERR_NO_MEM;

    const uint8_t page = (uint8_t)(base_pid / 0x20U);
    s_ecus[index].known_pages[page] = 1U;
    for (uint8_t bit = 0; bit < 32U; ++bit) {
        const uint16_t pid_value = (uint16_t)base_pid + bit + 1U;
        if (pid_value > 0xFFU) continue;
        const uint8_t pid = (uint8_t)pid_value;
        const bool supported = (bitmap[bit / 8U] & (uint8_t)(0x80U >> (bit % 8U))) != 0U;
        if (supported) s_ecus[index].supported[pid / 8U] |= (uint8_t)(1U << (pid % 8U));
        else s_ecus[index].supported[pid / 8U] &= (uint8_t)~(1U << (pid % 8U));
        const int descriptor_index = catalog_index(pid);
        if (supported && descriptor_index < 0 && bit != 31U) {
            ESP_LOGW(TAG, "ECU 0x%0*lX supports standard PID 0x%02X; decoder not implemented",
                     extended_id ? 8 : 3, (unsigned long)ecu_id, pid);
        }
        if (descriptor_index >= 0) {
            parameter_value_t *value = &s_ecus[index].values[descriptor_index];
            if (!supported && value->validity != PARAM_VALIDITY_VALID) {
                value->validity = PARAM_VALIDITY_UNSUPPORTED;
                value->pid = pid;
                value->ecu_id = ecu_id;
                value->extended_id = extended_id;
            } else if (supported && value->validity == PARAM_VALIDITY_UNSUPPORTED) {
                value->validity = PARAM_VALIDITY_UNAVAILABLE;
            }
        }
    }
    ESP_LOGI(TAG, "PID bitmap cached ECU=%s:0x%0*lX page=0x%02X continuation=%s",
             extended_id ? "EXT" : "STD", extended_id ? 8 : 3,
             (unsigned long)ecu_id, base_pid,
             parameter_manager_page_has_continuation(bitmap) ? "yes" : "no");
    return ESP_OK;
}

bool parameter_manager_page_has_continuation(const uint8_t bitmap[4])
{
    return bitmap != NULL && (bitmap[3] & 0x01U) != 0U;
}

bool parameter_manager_pid_support_known(uint32_t ecu_id, bool extended_id,
                                         uint8_t pid)
{
    int index = find_ecu(ecu_id, extended_id, false);
    if (index < 0) return false;
    const uint8_t page = (uint8_t)(pid / 0x20U);
    return (s_ecus[index].known_pages[page] != 0U);
}

bool parameter_manager_pid_supported(uint32_t ecu_id, bool extended_id,
                                     uint8_t pid)
{
    int index = find_ecu(ecu_id, extended_id, false);
    if (index < 0 || pid == 0U) return false;
    return (s_ecus[index].supported[pid / 8U] & (uint8_t)(1U << (pid % 8U))) != 0U;
}

bool parameter_manager_any_ecu_supports(uint8_t pid)
{
    for (size_t i = 0; i < PARAMETER_MANAGER_MAX_ECUS; ++i) {
        if (s_ecus[i].used && parameter_manager_pid_supported(s_ecus[i].id,
                                                               s_ecus[i].extended,
                                                               pid)) return true;
    }
    return false;
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t read_u32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | data[3];
}

esp_err_t parameter_manager_decode_obd(uint32_t ecu_id, bool extended_id,
                                       uint8_t pid, const uint8_t *data,
                                       size_t length, int64_t timestamp_us,
                                       parameter_value_t *value)
{
    if (data == NULL || value == NULL) return ESP_ERR_INVALID_ARG;
    const int index = catalog_index(pid);
    if (index < 0) return ESP_ERR_NOT_SUPPORTED;
    const parameter_descriptor_t *descriptor = &CATALOG[index];
    if (length != descriptor->response_length || length > sizeof(value->raw)) {
        return ESP_ERR_INVALID_SIZE;
    }

    float decoded;
    switch (descriptor->decoder) {
    case PARAM_DECODER_PERCENT_A:
    case PARAM_DECODER_ETHANOL_PERCENT_A:
        decoded = (float)data[0] * 100.0f / 255.0f;
        break;
    case PARAM_DECODER_PRESSURE_A_TIMES_3:
        decoded = (float)data[0] * 3.0f;
        break;
    case PARAM_DECODER_FUEL_TRIM_A:
    case PARAM_DECODER_EGR_ERROR_A:
        decoded = ((float)data[0] - 128.0f) * 100.0f / 128.0f;
        break;
    case PARAM_DECODER_COOLANT_A_MINUS_40:
    case PARAM_DECODER_TEMPERATURE_A_MINUS_40:
        decoded = (float)data[0] - 40.0f;
        break;
    case PARAM_DECODER_MAP_KPA:
    case PARAM_DECODER_SPEED_KMH:
    case PARAM_DECODER_FUEL_LEVEL_PERCENT:
        decoded = (descriptor->decoder == PARAM_DECODER_FUEL_LEVEL_PERCENT)
            ? (float)data[0] * 100.0f / 255.0f : (float)data[0];
        break;
    case PARAM_DECODER_RPM_AB_DIV_4:
        decoded = (float)read_u16(data) / 4.0f;
        break;
    case PARAM_DECODER_MAF_AB_DIV_100:
        decoded = (float)read_u16(data) / 100.0f;
        break;
    case PARAM_DECODER_IGNITION_A_DIV_2_MINUS_64:
        decoded = (float)data[0] / 2.0f - 64.0f;
        break;
    case PARAM_DECODER_U16_AB:
    case PARAM_DECODER_DISTANCE_AB:
        decoded = (float)read_u16(data);
        break;
    case PARAM_DECODER_VOLTAGE_AB_DIV_1000:
        decoded = (float)read_u16(data) / 1000.0f;
        break;
    case PARAM_DECODER_CATALYST_TEMP_AB_DIV_10_MINUS_40:
        decoded = (float)read_u16(data) / 10.0f - 40.0f;
        break;
    case PARAM_DECODER_TORQUE_PERCENT_A_MINUS_125:
        decoded = (float)data[0] - 125.0f;
        break;
    case PARAM_DECODER_TORQUE_REFERENCE_AB:
        decoded = (float)read_u16(data);
        break;
    case PARAM_DECODER_FUEL_RAIL_PRESSURE_AB:
        decoded = (float)read_u16(data) * 0.079f;
        break;
    case PARAM_DECODER_ABSOLUTE_LOAD_AB:
        decoded = (float)read_u16(data) * 100.0f / 255.0f;
        break;
    case PARAM_DECODER_EQUIVALENCE_RATIO_AB:
        decoded = (float)read_u16(data) * 2.0f / 65536.0f;
        break;
    case PARAM_DECODER_INJECTION_TIMING_AB:
        decoded = (float)read_u16(data) / 128.0f - 210.0f;
        break;
    case PARAM_DECODER_FUEL_RATE_AB:
        decoded = (float)read_u16(data) * 0.05f;
        break;
    case PARAM_DECODER_TURBO_RPM_AB_TIMES_10:
        decoded = (float)read_u16(data) * 10.0f;
        break;
    case PARAM_DECODER_ODOMETER_ABCD_DIV_10:
        decoded = (float)read_u32(data) / 10.0f;
        break;
    case PARAM_DECODER_EVAP_PRESSURE_A_DIV_200:
        decoded = (float)data[0] / 200.0f;
        break;
    case PARAM_DECODER_PRESSURE_AB_TIMES_10:
        decoded = (float)read_u16(data) * 10.0f;
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (decoded < descriptor->minimum || decoded > descriptor->maximum) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    int ecu_index = find_ecu(ecu_id, extended_id, true);
    if (ecu_index < 0) return ESP_ERR_NO_MEM;
    *value = (parameter_value_t) {
        .ecu_id = ecu_id,
        .extended_id = extended_id,
        .pid = pid,
        .raw_length = (uint8_t)length,
        .value = decoded,
        .timestamp_us = timestamp_us,
        .validity = PARAM_VALIDITY_VALID,
    };
    memcpy(value->raw, data, length);
    s_ecus[ecu_index].values[index] = *value;
    return ESP_OK;
}

void parameter_manager_record_timeout(uint8_t pid, int64_t timestamp_us)
{
    const int index = catalog_index(pid);
    if (index < 0) return;
    s_global_timeouts[index] = (parameter_value_t) {
        .pid = pid,
        .timestamp_us = timestamp_us,
        .validity = PARAM_VALIDITY_TIMEOUT,
    };
}

void parameter_manager_mark_stale(int64_t now_us)
{
    for (size_t ecu = 0; ecu < PARAMETER_MANAGER_MAX_ECUS; ++ecu) {
        if (!s_ecus[ecu].used) continue;
        for (size_t i = 0; i < sizeof(CATALOG) / sizeof(CATALOG[0]); ++i) {
            parameter_value_t *value = &s_ecus[ecu].values[i];
            const int64_t stale_after_us = (int64_t)CATALOG[i].update_period_ms * 3000LL;
            if (value->validity == PARAM_VALIDITY_VALID &&
                now_us - value->timestamp_us >= stale_after_us) {
                value->validity = PARAM_VALIDITY_STALE;
            }
        }
    }
}

bool parameter_manager_get_value(uint32_t ecu_id, bool extended_id, uint8_t pid,
                                 parameter_value_t *value)
{
    if (value == NULL) return false;
    const int ecu_index = find_ecu(ecu_id, extended_id, false);
    const int index = catalog_index(pid);
    if (ecu_index < 0 || index < 0) return false;
    *value = s_ecus[ecu_index].values[index];
    return value->validity != PARAM_VALIDITY_UNAVAILABLE;
}

bool parameter_manager_get_latest_value(uint8_t pid, parameter_value_t *value)
{
    if (value == NULL) return false;
    const int catalog_entry = catalog_index(pid);
    if (catalog_entry < 0) return false;

    const parameter_value_t *timeout = &s_global_timeouts[catalog_entry];
    bool found = timeout->validity == PARAM_VALIDITY_TIMEOUT;
    if (found) *value = *timeout;
    for (size_t ecu = 0; ecu < PARAMETER_MANAGER_MAX_ECUS; ++ecu) {
        if (!s_ecus[ecu].used) continue;
        const parameter_value_t *candidate = &s_ecus[ecu].values[catalog_entry];
        if (candidate->validity == PARAM_VALIDITY_UNAVAILABLE ||
            candidate->validity == PARAM_VALIDITY_UNSUPPORTED) continue;
        if (!found || candidate->timestamp_us > value->timestamp_us) {
            *value = *candidate;
            found = true;
        }
    }
    return found;
}

const char *parameter_manager_validity_name(parameter_validity_t validity)
{
    switch (validity) {
    case PARAM_VALIDITY_VALID: return "VALID";
    case PARAM_VALIDITY_UNSUPPORTED: return "UNSUPPORTED";
    case PARAM_VALIDITY_TIMEOUT: return "TIMEOUT";
    case PARAM_VALIDITY_STALE: return "STALE";
    case PARAM_VALIDITY_INVALID: return "INVALID";
    case PARAM_VALIDITY_UNAVAILABLE:
    default: return "UNAVAILABLE";
    }
}
