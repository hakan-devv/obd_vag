#ifndef DID_PROFILE_H
#define DID_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "diag_ecu_profile.h"
#include "uds_did_catalog.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VIN/ECU based profile grouping and supported-DID tracking. Pure logic: no CAN,
 * UDS or display access, so it can be unit tested on the host image. The
 * manual `d` scan never uses this module.
 */

typedef enum {
    DID_BRAND_UNKNOWN = 0,
    DID_BRAND_VW,
    DID_BRAND_SEAT,
    DID_BRAND_SKODA,
    DID_BRAND_AUDI,
    DID_BRAND_BMW,
} did_brand_t;

#define DID_PROFILE_POOL_MAX 10U

did_brand_t did_profile_brand_from_vin(const char *vin);
const char *did_profile_brand_name(did_brand_t brand);
/* Candidate catalog profile ids, most likely first. `f197` is the ECU system name. */
size_t did_profile_pool(did_brand_t brand, const char *f197,
                        const char *out[DID_PROFILE_POOL_MAX]);
/* Engine family label from the F197 engine code, or NULL when the code is unknown. */
const char *did_profile_engine_family(const char *f197);
/* Human readable decode of an identification DID payload (VIN, part numbers...). */
void did_profile_decode_identity(uint16_t did, const uint8_t *data, size_t length,
                                 char *out, size_t out_size);

#define DID_POLL_MAX_ENTRIES 320U
#define DID_POLL_DEFAULT_FAST_MS 500U
#define DID_POLL_DEFAULT_SLOW_MS 1000U
#define DID_POLL_NOT_READY_MS 2000U
#define DID_POLL_MAX_TIMEOUTS 2U
#define DID_POLL_SILENT_BACKOFF_MS 5000U
/* Identification DIDs (VEH layer) beyond the fixed core list: every catalog entry of
 * the "vag-uds-identity" profile. `index` walks core list first; returns 0 at the end. */
uint16_t did_profile_identity_did_at(size_t index, const uint16_t *core, size_t core_count);

typedef enum {
    DID_POLL_UNTESTED = 0,
    DID_POLL_SUPPORTED,
    DID_POLL_NOT_READY, /* ECU knows the DID but has no data yet (0x78/0x22...) */
    DID_POLL_EXCLUDED,  /* NRC 0x31 and other hard rejections: never asked again */
} did_poll_state_t;

typedef struct {
    const uds_did_descriptor_t *descriptor;
    did_poll_state_t state;
    uint8_t timeouts;
    uint8_t last_nrc;
    uint32_t interval_ms;
    int64_t next_due_us;
} did_poll_entry_t;

typedef struct {
    did_poll_entry_t entries[DID_POLL_MAX_ENTRIES];
    size_t count;
} did_poll_set_t;

void did_poll_set_init(did_poll_set_t *set);
/* Adds decodable, non-session catalog DIDs of the pools, deduplicated by service+DID. */
size_t did_poll_set_build(did_poll_set_t *set, diag_ecu_role_t role,
                          const char *const *profiles, size_t profile_count);
bool did_poll_contains(const did_poll_set_t *set, uint16_t did, uint8_t service);
int did_poll_next_untested(const did_poll_set_t *set);
int did_poll_next_due(const did_poll_set_t *set, int64_t now_us);
size_t did_poll_count(const did_poll_set_t *set, did_poll_state_t state);
void did_poll_record_positive(did_poll_set_t *set, size_t index, int64_t now_us);
void did_poll_record_negative(did_poll_set_t *set, size_t index, uint8_t nrc,
                              int64_t now_us);
void did_poll_record_pending(did_poll_set_t *set, size_t index, int64_t now_us);
void did_poll_record_timeout(did_poll_set_t *set, size_t index, int64_t now_us);
bool did_poll_nrc_means_unsupported(uint8_t nrc);

#ifdef __cplusplus
}
#endif

#endif
