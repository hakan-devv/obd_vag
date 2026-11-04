#ifndef DIAG_ECU_PROFILE_H
#define DIAG_ECU_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DIAG_ECU_ROLE_ENGINE = 0,
    DIAG_ECU_ROLE_TRANSMISSION,
    DIAG_ECU_ROLE_HALDEX,
    DIAG_ECU_ROLE_OTHER,
} diag_ecu_role_t;

typedef enum {
    DIAG_VALIDATION_EXPERIMENTAL = 0,
    DIAG_VALIDATION_SOURCE_BACKED,
    DIAG_VALIDATION_VEHICLE_VERIFIED,
} diag_validation_level_t;

typedef struct {
    const char *profile_id;
    diag_ecu_role_t role;
    uint32_t tx_id;
    uint32_t rx_id;
    bool tx_extended;
    bool rx_extended;
    bool active_diagnostics_allowed;
    bool require_profile_match;
    const char *source_url;
    diag_validation_level_t validation;
} diag_ecu_route_t;

const diag_ecu_route_t *diag_ecu_route_table(size_t *count);
diag_ecu_role_t diag_ecu_role_for_rx_id(uint32_t rx_id, bool extended_id);
bool diag_ecu_route_is_known(uint32_t tx_id, uint32_t rx_id, bool tx_extended,
                            bool rx_extended, diag_ecu_role_t role);
bool diag_ecu_route_is_allowed(uint32_t tx_id, uint32_t rx_id, bool tx_extended,
                               bool rx_extended, diag_ecu_role_t role);
bool diag_ecu_profile_is_known(const char *profile_id);

#ifdef __cplusplus
}
#endif

#endif
