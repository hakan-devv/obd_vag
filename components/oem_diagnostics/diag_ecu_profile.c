#include "diag_ecu_profile.h"
#include "uds_did_catalog.h"

#include <string.h>

/*
 * Routes describe standard physical addressing observed/expected for common
 * OBD responders. They do not cause a bitrate or CAN-ID-width switch: callers
 * must use the live s_can_config selected and verified by can_discovery.
 * OpenHaldex-Edge is documented separately but is deliberately not activated
 * by this OBD responder route table (0x70F -> 0x779 is not in the current
 * OBD ISO-TP receive path).
 */
static const diag_ecu_route_t s_routes[] = {
    {
        .profile_id = "vag-11bit-engine-route",
        .role = DIAG_ECU_ROLE_ENGINE,
        .tx_id = 0x7E0U, .rx_id = 0x7E8U,
        .tx_extended = false, .rx_extended = false,
        .active_diagnostics_allowed = true,
        .require_profile_match = false,
        .source_url = "ISO 15765-4 physical OBD addressing; route is selected only after live bus verification",
        .validation = DIAG_VALIDATION_SOURCE_BACKED,
    },
    {
        .profile_id = "vag-11bit-transmission-route",
        .role = DIAG_ECU_ROLE_TRANSMISSION,
        .tx_id = 0x7E1U, .rx_id = 0x7E9U,
        .tx_extended = false, .rx_extended = false,
        .active_diagnostics_allowed = true,
        .require_profile_match = false,
        .source_url = "Common physical OBD addressing; ECU role must be confirmed from response ID/profile",
        .validation = DIAG_VALIDATION_SOURCE_BACKED,
    },
    {
        .profile_id = "openhaldex-edge-gen5-route",
        .role = DIAG_ECU_ROLE_HALDEX,
        .tx_id = 0x70FU, .rx_id = 0x779U,
        .tx_extended = false, .rx_extended = false,
        .active_diagnostics_allowed = false,
        .require_profile_match = true,
        .source_url = "https://github.com/Kile-Thomson/OpenHaldex-Edge/blob/main/CHANGELOG.md",
        .validation = DIAG_VALIDATION_EXPERIMENTAL,
    },
};

const diag_ecu_route_t *diag_ecu_route_table(size_t *count)
{
    if (count != NULL) *count = sizeof(s_routes) / sizeof(s_routes[0]);
    return s_routes;
}

diag_ecu_role_t diag_ecu_role_for_rx_id(uint32_t rx_id, bool extended_id)
{
    /* 29-bit OBD physical response: 18DAF1xx. The last byte is the ECU address,
     * so do not hard-code one particular ECU address (e.g. 0x10). The route is
     * only used after obd_client has verified a valid 41 00 on this bus. */
    if (extended_id && rx_id >= 0x18DAF100U && rx_id <= 0x18DAF1FFU) {
        return DIAG_ECU_ROLE_ENGINE;
    }
    const size_t count = sizeof(s_routes) / sizeof(s_routes[0]);
    for (size_t i = 0; i < count; ++i) {
        const diag_ecu_route_t *entry = &s_routes[i];
        if (entry->rx_extended == extended_id && entry->rx_id == rx_id) return entry->role;
    }
    return DIAG_ECU_ROLE_OTHER;
}

bool diag_ecu_route_is_known(uint32_t tx_id, uint32_t rx_id, bool tx_extended,
                             bool rx_extended, diag_ecu_role_t role)
{
    if (tx_extended && rx_extended && role == DIAG_ECU_ROLE_ENGINE &&
        rx_id >= 0x18DAF100U && rx_id <= 0x18DAF1FFU) {
        const uint32_t expected_tx = 0x18DA0000U | ((rx_id & 0xFFU) << 8U) | 0xF1U;
        if (tx_id == expected_tx) return true;
    }
    const size_t count = sizeof(s_routes) / sizeof(s_routes[0]);
    for (size_t i = 0; i < count; ++i) {
        const diag_ecu_route_t *entry = &s_routes[i];
        if (entry->role == role && entry->tx_extended == tx_extended &&
            entry->rx_extended == rx_extended && entry->tx_id == tx_id &&
            entry->rx_id == rx_id) return true;
    }
    return false;
}

bool diag_ecu_route_is_allowed(uint32_t tx_id, uint32_t rx_id, bool tx_extended,
                               bool rx_extended, diag_ecu_role_t role)
{
    /* This is a physical route derived from the exact observed OBD response ID.
     * It is not a request to change bitrate or identifier width. */
    if (tx_extended && rx_extended && role == DIAG_ECU_ROLE_ENGINE &&
        rx_id >= 0x18DAF100U && rx_id <= 0x18DAF1FFU) {
        const uint32_t expected_tx = 0x18DA0000U | ((rx_id & 0xFFU) << 8U) | 0xF1U;
        return tx_id == expected_tx;
    }
    const size_t count = sizeof(s_routes) / sizeof(s_routes[0]);
    for (size_t i = 0; i < count; ++i) {
        const diag_ecu_route_t *entry = &s_routes[i];
        if (entry->role != role || entry->tx_extended != tx_extended ||
            entry->rx_extended != rx_extended || entry->tx_id != tx_id ||
            entry->rx_id != rx_id) continue;
        return entry->active_diagnostics_allowed &&
               (!entry->require_profile_match || entry->validation >= DIAG_VALIDATION_SOURCE_BACKED);
    }
    return false;
}

bool diag_ecu_profile_is_known(const char *profile_id)
{
    if (profile_id == NULL || profile_id[0] == '\0') return false;
    size_t count = 0U;
    const uds_did_descriptor_t *catalog = uds_did_catalog_table(&count);
    for (size_t i = 0U; i < count; ++i) {
        if (catalog[i].profile_id != NULL &&
            strcmp(catalog[i].profile_id, profile_id) == 0 &&
            catalog[i].manual_probe_allowed &&
            (catalog[i].role == DIAG_ECU_ROLE_ENGINE ||
             catalog[i].role == DIAG_ECU_ROLE_TRANSMISSION)) {
            return true;
        }
    }
    return false;
}
