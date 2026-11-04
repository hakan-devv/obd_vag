#ifndef UDS_DID_CATALOG_H
#define UDS_DID_CATALOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "diag_ecu_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UDS_DID_VALIDATION_EXPERIMENTAL = 0,
    UDS_DID_VALIDATION_SOURCE_BACKED,
    UDS_DID_VALIDATION_SAMPLE_CONFIRMED,
    UDS_DID_VALIDATION_VEHICLE_VERIFIED,
} uds_did_validation_level_t;

typedef enum {
    UDS_DID_DECODE_RAW = 0,
    UDS_DID_DECODE_U8,
    UDS_DID_DECODE_S8,
    UDS_DID_DECODE_U16_BE,
    UDS_DID_DECODE_S16_BE,
    UDS_DID_DECODE_U16_LE,
    UDS_DID_DECODE_S16_LE,
    UDS_DID_DECODE_U32_BE,
    UDS_DID_DECODE_S32_BE,
    UDS_DID_DECODE_ASCII,
} uds_did_decode_kind_t;

typedef struct {
    uint16_t did;
    /* Request metadata consumed by obd_client's DID scanner. */
    uint8_t service;
    uint8_t subfunction;
    const char *decoder;
    diag_ecu_role_t role;
    /* Model/ECU applicability, not a generic VAG-wide profile. */
    const char *profile_id;
    const char *name;
    const char *unit;
    const char *description;
    uint8_t min_length;
    uint8_t max_length;
    uint8_t data_offset;
    uds_did_decode_kind_t decode_kind;
    double scale;
    double offset;
    bool has_engineering_range;
    double engineering_min;
    double engineering_max;
    /* Only explicit user-invoked `d` scans may use these entries. */
    bool manual_probe_allowed;
    /* Reserved for a future exact ECU fingerprint + vehicle verification. */
    bool auto_query_allowed;
    bool session_required;
    uint32_t poll_interval_ms;
    uint8_t priority;
    uds_did_validation_level_t validation;
    const char *source_url;
} uds_did_descriptor_t;

const uds_did_descriptor_t *uds_did_catalog_table(size_t *count);
const uds_did_descriptor_t *uds_did_catalog_find(uint16_t did, diag_ecu_role_t role);
bool uds_did_catalog_is_auto_allowed(uint16_t did, diag_ecu_role_t role);
bool uds_did_catalog_is_valid(uint16_t did, diag_ecu_role_t role);
bool uds_did_catalog_decode(const uds_did_descriptor_t *descriptor,
                            const uint8_t *data, size_t length,
                            double *value);
bool uds_did_catalog_is_query_allowed(uint16_t did,
                                      diag_ecu_role_t role,
                                      const char *profile_id,
                                      bool session_ready,
                                      bool allow_experimental,
                                      bool manual_probe);

#ifdef __cplusplus
}
#endif

#endif
