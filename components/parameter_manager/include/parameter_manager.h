#ifndef PARAMETER_MANAGER_H
#define PARAMETER_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAMETER_MANAGER_MAX_ECUS 8U
#define PARAMETER_MANAGER_MAX_PIDS 0xE0U

typedef enum {
    PARAM_SOURCE_STANDARD_PID = 0,
    PARAM_SOURCE_UDS_DID,
} parameter_source_t;

typedef enum {
    PARAM_VALIDITY_UNAVAILABLE = 0,
    PARAM_VALIDITY_VALID,
    PARAM_VALIDITY_UNSUPPORTED,
    PARAM_VALIDITY_TIMEOUT,
    PARAM_VALIDITY_STALE,
    PARAM_VALIDITY_INVALID,
} parameter_validity_t;

typedef enum {
    PARAM_DECODER_PERCENT_A = 0,
    PARAM_DECODER_FUEL_TRIM_A,
    PARAM_DECODER_COOLANT_A_MINUS_40,
    PARAM_DECODER_MAP_KPA,
    PARAM_DECODER_RPM_AB_DIV_4,
    PARAM_DECODER_SPEED_KMH,
    PARAM_DECODER_TEMPERATURE_A_MINUS_40,
    PARAM_DECODER_MAF_AB_DIV_100,
    PARAM_DECODER_IGNITION_A_DIV_2_MINUS_64,
    PARAM_DECODER_U16_AB,
    PARAM_DECODER_FUEL_LEVEL_PERCENT,
    PARAM_DECODER_VOLTAGE_AB_DIV_1000,
    PARAM_DECODER_CATALYST_TEMP_AB_DIV_10_MINUS_40,
    PARAM_DECODER_TORQUE_PERCENT_A_MINUS_125,
    PARAM_DECODER_TORQUE_REFERENCE_AB,
    PARAM_DECODER_PRESSURE_A_TIMES_3,
    PARAM_DECODER_EGR_ERROR_A,
    PARAM_DECODER_DISTANCE_AB,
    PARAM_DECODER_FUEL_RAIL_PRESSURE_AB,
    PARAM_DECODER_EVAP_PRESSURE_AB,
    PARAM_DECODER_ABSOLUTE_LOAD_AB,
    PARAM_DECODER_EQUIVALENCE_RATIO_AB,
    PARAM_DECODER_ETHANOL_PERCENT_A,
    PARAM_DECODER_INJECTION_TIMING_AB,
    PARAM_DECODER_FUEL_RATE_AB,
    PARAM_DECODER_TURBO_RPM_AB_DIV_4,
    PARAM_DECODER_ODOMETER_ABCD_DIV_10,
    PARAM_DECODER_PRESSURE_AB_TIMES_10,
} parameter_decoder_t;

typedef struct {
    uint8_t pid;
    const char *name;
    const char *unit;
    uint8_t response_length;
    parameter_decoder_t decoder;
    float minimum;
    float maximum;
    uint16_t update_period_ms;
    uint8_t priority;
} parameter_descriptor_t;

typedef struct {
    uint32_t ecu_id;
    bool extended_id;
    uint8_t pid;
    uint8_t raw[8];
    uint8_t raw_length;
    float value;
    int64_t timestamp_us;
    parameter_validity_t validity;
} parameter_value_t;

void parameter_manager_init(void);
const parameter_descriptor_t *parameter_manager_catalog(size_t *count);
esp_err_t parameter_manager_record_supported_page(uint32_t ecu_id, bool extended_id,
                                                  uint8_t base_pid,
                                                  const uint8_t bitmap[4]);
bool parameter_manager_pid_support_known(uint32_t ecu_id, bool extended_id,
                                         uint8_t pid);
bool parameter_manager_pid_supported(uint32_t ecu_id, bool extended_id,
                                     uint8_t pid);
bool parameter_manager_page_has_continuation(const uint8_t bitmap[4]);
bool parameter_manager_any_ecu_supports(uint8_t pid);
esp_err_t parameter_manager_decode_obd(uint32_t ecu_id, bool extended_id,
                                       uint8_t pid, const uint8_t *data,
                                       size_t length, int64_t timestamp_us,
                                       parameter_value_t *value);
void parameter_manager_record_timeout(uint8_t pid, int64_t timestamp_us);
void parameter_manager_mark_stale(int64_t now_us);
bool parameter_manager_get_value(uint32_t ecu_id, bool extended_id, uint8_t pid,
                                 parameter_value_t *value);
bool parameter_manager_get_latest_value(uint8_t pid, parameter_value_t *value);
const char *parameter_manager_validity_name(parameter_validity_t validity);

#ifdef __cplusplus
}
#endif

#endif
