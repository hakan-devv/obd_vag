#ifndef OEM_DID_SCAN_H
#define OEM_DID_SCAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "diag_ecu_profile.h"
#include "uds_did_catalog.h"

#ifdef __cplusplus
extern "C" {
#endif

const uds_did_descriptor_t *oem_did_scan_candidates_for_role(diag_ecu_role_t role,
                                                             size_t *count,
                                                             bool allow_experimental);
diag_ecu_role_t oem_did_scan_role_for_index(uint8_t ecu_index);
const char *oem_did_scan_role_name(diag_ecu_role_t role);

#ifdef __cplusplus
}
#endif

#endif
