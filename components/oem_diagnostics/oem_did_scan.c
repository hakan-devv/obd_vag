#include "oem_did_scan.h"

#include <string.h>

const uds_did_descriptor_t *oem_did_scan_candidates_for_role(diag_ecu_role_t role,
                                                             size_t *count,
                                                             bool allow_experimental)
{
    static uds_did_descriptor_t filtered[256];
    size_t used = 0U;
    size_t total = 0U;
    const uds_did_descriptor_t *catalog = uds_did_catalog_table(&total);
    for (size_t i = 0; i < total && used < (sizeof(filtered) / sizeof(filtered[0])); ++i) {
        if (catalog[i].role != role || !catalog[i].manual_probe_allowed) continue;
        if (!allow_experimental && catalog[i].validation < UDS_DID_VALIDATION_SOURCE_BACKED) continue;
        filtered[used++] = catalog[i];
    }
    if (count != NULL) *count = used;
    return used != 0U ? filtered : NULL;
}

diag_ecu_role_t oem_did_scan_role_for_index(uint8_t ecu_index)
{
    /* Legacy index mapping retained for callers without an observed RX ID. */
    if (ecu_index == 0U) return DIAG_ECU_ROLE_ENGINE;
    if (ecu_index == 1U) return DIAG_ECU_ROLE_TRANSMISSION;
    return DIAG_ECU_ROLE_OTHER;
}

const char *oem_did_scan_role_name(diag_ecu_role_t role)
{
    switch (role) {
    case DIAG_ECU_ROLE_ENGINE: return "ENGINE";
    case DIAG_ECU_ROLE_TRANSMISSION: return "TRANSMISSION";
    case DIAG_ECU_ROLE_HALDEX: return "HALDEX";
    default: return "OTHER";
    }
}
