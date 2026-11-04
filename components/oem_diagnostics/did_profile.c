#include "did_profile.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static const char *const s_vag_engine_profiles[] = {
    "vw-up-engine",     "vw-jetta-engine",    "vw-taigo-engine",
    "vw-touareg-engine", "seat-mii-engine",   "seat-ateca-engine",
    /* Last on purpose: model-specific entries win when the same DID is defined twice. */
    "vag-engine-community",
};

static bool wmi_is(const char *vin, const char *wmi)
{
    return strncmp(vin, wmi, 3U) == 0;
}

did_brand_t did_profile_brand_from_vin(const char *vin)
{
    if (vin == NULL || strnlen(vin, 4U) < 3U) return DID_BRAND_UNKNOWN;
    static const char *const bmw[] = {"WBA", "WBS", "WBY"};
    static const char *const audi[] = {"WAU", "WUA", "TRU"};
    static const char *const vw[] = {"WVW", "WVG", "WV1", "WV2", "WV3",
                                      "3VW", "1VW", "9BW", "AAV", "LFV", "8AW"};
    for (size_t i = 0; i < sizeof(bmw) / sizeof(bmw[0]); ++i)
        if (wmi_is(vin, bmw[i])) return DID_BRAND_BMW;
    for (size_t i = 0; i < sizeof(audi) / sizeof(audi[0]); ++i)
        if (wmi_is(vin, audi[i])) return DID_BRAND_AUDI;
    if (wmi_is(vin, "TMB")) return DID_BRAND_SKODA;
    if (wmi_is(vin, "VSS")) return DID_BRAND_SEAT;
    for (size_t i = 0; i < sizeof(vw) / sizeof(vw[0]); ++i)
        if (wmi_is(vin, vw[i])) return DID_BRAND_VW;
    return DID_BRAND_UNKNOWN;
}

const char *did_profile_brand_name(did_brand_t brand)
{
    switch (brand) {
    case DID_BRAND_VW: return "VW";
    case DID_BRAND_SEAT: return "SEAT";
    case DID_BRAND_SKODA: return "SKODA";
    case DID_BRAND_AUDI: return "AUDI";
    case DID_BRAND_BMW: return "BMW";
    default: return "UNKNOWN";
    }
}

static bool f197_has(const char *f197, const char *token)
{
    return f197 != NULL && strstr(f197, token) != NULL;
}

/* Engine codes are from public VAG engine-code lists and are UNVERIFIED on real cars
 * until confirmed by `d` scans; the family is only a label and does not narrow the pool. */
const char *did_profile_engine_family(const char *f197)
{
    static const struct { const char *code; const char *family; } table[] = {
        {"CAXA", "EA111-1.4TSI"}, {"CAVD", "EA111-1.4TSI"}, {"CTHD", "EA111-1.4TSI"},
        {"CDAA", "EA888-Gen1/2"}, {"CAEB", "EA888-Gen2"}, {"CBFA", "EA888-Gen2"},
        {"CJSA", "EA888-Gen3"}, {"CJXC", "EA888-Gen3"}, {"CHHB", "EA888-Gen3"},
        {"CNCD", "EA888-Gen3"}, {"DKZA", "EA888-Gen3"}, {"DNUE", "EA888-Gen4"},
        {"CJZA", "EA211"}, {"CHPA", "EA211"}, {"CZCA", "EA211"}, {"CYVB", "EA211"},
        {"DADA", "EA211-EVO"},
    };
    if (f197 == NULL) return NULL;
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); ++i) {
        if (strstr(f197, table[i].code) != NULL) return table[i].family;
    }
    return NULL;
}

size_t did_profile_pool(did_brand_t brand, const char *f197,
                        const char *out[DID_PROFILE_POOL_MAX])
{
    size_t n = 0U;
    const size_t vag_count = sizeof(s_vag_engine_profiles) / sizeof(s_vag_engine_profiles[0]);
    bool vag_family = brand == DID_BRAND_VW || brand == DID_BRAND_SEAT ||
                      brand == DID_BRAND_SKODA || brand == DID_BRAND_AUDI;

    if (brand == DID_BRAND_BMW) {
        /* BMW 0x2C reads are only sent to an ECU that identifies itself as N47/DDE7. */
        if (f197_has(f197, "N47") || f197_has(f197, "DDE7")) out[n++] = "bmw-n47-dde7";
        return n;
    }
    if (!vag_family && brand == DID_BRAND_UNKNOWN &&
        (f197_has(f197, "MED17") || f197_has(f197, "MED 17") ||
         f197_has(f197, "ME17") || f197_has(f197, "MEB") || f197_has(f197, "VAG"))) {
        vag_family = true;
    }
    if (!vag_family) return 0U;

    /* The brand's own entries first: the same DID means different things per model. */
    if (brand == DID_BRAND_SEAT) {
        out[n++] = "seat-mii-engine";
        out[n++] = "seat-ateca-engine";
    } else if (brand == DID_BRAND_SKODA) {
        out[n++] = "skoda-octavia-engine";
    } else if (brand == DID_BRAND_AUDI) {
        out[n++] = "audi-uds-powertrain-candidates";
    }
    for (size_t i = 0; i < vag_count && n < DID_PROFILE_POOL_MAX; ++i) {
        bool dup = false;
        for (size_t j = 0; j < n; ++j) {
            if (strcmp(out[j], s_vag_engine_profiles[i]) == 0) dup = true;
        }
        if (!dup) out[n++] = s_vag_engine_profiles[i];
    }
    return n;
}

uint16_t did_profile_identity_did_at(size_t index, const uint16_t *core, size_t core_count)
{
    if (core != NULL && index < core_count) return core[index];
    size_t skip = index - (core != NULL ? core_count : 0U);
    size_t count = 0U;
    const uds_did_descriptor_t *catalog = uds_did_catalog_table(&count);
    for (size_t i = 0; catalog != NULL && i < count; ++i) {
        const uds_did_descriptor_t *d = &catalog[i];
        if (d->role != DIAG_ECU_ROLE_ENGINE || d->service != 0x22U ||
            d->profile_id == NULL || strcmp(d->profile_id, "vag-uds-identity") != 0) continue;
        bool in_core = false;
        for (size_t k = 0; core != NULL && k < core_count; ++k) {
            if (core[k] == d->did) in_core = true;
        }
        if (in_core) continue;
        if (skip == 0U) return d->did;
        --skip;
    }
    return 0U;
}

void did_profile_decode_identity(uint16_t did, const uint8_t *data, size_t length,
                                 char *out, size_t out_size)
{
    if (out == NULL || out_size == 0U) return;
    out[0] = '\0';
    if (data == NULL || length == 0U) return;

    /* ISO 14229 dates: 3 BCD bytes YY MM DD. */
    if ((did == 0xF18BU || did == 0xF199U || did == 0xF19BU || did == 0xF19DU) && length == 3U &&
        (data[0] & 0x0FU) < 10U && (data[0] >> 4) < 10U && (data[1] & 0x0FU) < 10U &&
        (data[1] >> 4) < 10U && (data[2] & 0x0FU) < 10U && (data[2] >> 4) < 10U) {
        (void)snprintf(out, out_size, "20%02X-%02X-%02X", data[0], data[1], data[2]);
        return;
    }
    if (did == 0xF186U && length == 1U) {
        (void)snprintf(out, out_size, "session 0x%02X%s", data[0],
                       data[0] == 0x01U ? " (default)" : "");
        return;
    }
    size_t trimmed = length;
    while (trimmed > 0U && (data[trimmed - 1U] == 0x00U || data[trimmed - 1U] == 0xFFU ||
                            data[trimmed - 1U] == 0x20U)) {
        --trimmed;
    }
    bool printable = trimmed > 0U;
    for (size_t i = 0; i < trimmed; ++i) {
        if (data[i] < 0x20U || data[i] > 0x7EU) printable = false;
    }
    if (printable && did != 0xF806U) {
        size_t n = trimmed < out_size - 1U ? trimmed : out_size - 1U;
        memcpy(out, data, n);
        out[n] = '\0';
        return;
    }
    /* Non-text identification data (CVN, BCD dates, flags): decoded as hex words. */
    size_t used = 0U;
    for (size_t i = 0; i < length && used + 3U < out_size; ++i) {
        used += (size_t)snprintf(out + used, out_size - used, i == 0U ? "%02X" : " %02X", data[i]);
    }
}

void did_poll_set_init(did_poll_set_t *set)
{
    if (set != NULL) memset(set, 0, sizeof(*set));
}

static uint32_t default_interval_ms(const uds_did_descriptor_t *d)
{
    if (d->poll_interval_ms != 0U) return d->poll_interval_ms;
    if (d->unit != NULL && (strcmp(d->unit, "degC") == 0 || strcmp(d->unit, "L") == 0)) {
        return DID_POLL_DEFAULT_SLOW_MS;
    }
    return DID_POLL_DEFAULT_FAST_MS;
}

size_t did_poll_set_build(did_poll_set_t *set, diag_ecu_role_t role,
                          const char *const *profiles, size_t profile_count)
{
    if (set == NULL) return 0U;
    did_poll_set_init(set);
    size_t catalog_count = 0U;
    const uds_did_descriptor_t *catalog = uds_did_catalog_table(&catalog_count);
    if (catalog == NULL || profiles == NULL) return 0U;

    for (size_t p = 0; p < profile_count; ++p) {
        for (size_t i = 0; i < catalog_count; ++i) {
            const uds_did_descriptor_t *d = &catalog[i];
            if (d->role != role || d->profile_id == NULL ||
                strcmp(d->profile_id, profiles[p]) != 0) continue;
            if (d->decode_kind == UDS_DID_DECODE_RAW || d->decode_kind == UDS_DID_DECODE_ASCII ||
                d->session_required) continue;
            bool dup = false;
            for (size_t k = 0; k < set->count; ++k) {
                if (set->entries[k].descriptor->did == d->did &&
                    set->entries[k].descriptor->service == d->service) {
                    dup = true;
                    break;
                }
            }
            if (dup || set->count >= DID_POLL_MAX_ENTRIES) continue;
            did_poll_entry_t *e = &set->entries[set->count++];
            e->descriptor = d;
            e->state = DID_POLL_UNTESTED;
            e->interval_ms = default_interval_ms(d);
        }
    }
    return set->count;
}

bool did_poll_contains(const did_poll_set_t *set, uint16_t did, uint8_t service)
{
    for (size_t i = 0; set != NULL && i < set->count; ++i) {
        if (set->entries[i].descriptor->did == did &&
            set->entries[i].descriptor->service == service) return true;
    }
    return false;
}

int did_poll_next_untested(const did_poll_set_t *set)
{
    for (size_t i = 0; set != NULL && i < set->count; ++i) {
        if (set->entries[i].state == DID_POLL_UNTESTED) return (int)i;
    }
    return -1;
}

int did_poll_next_due(const did_poll_set_t *set, int64_t now_us)
{
    int best = -1;
    int64_t best_due = 0;
    for (size_t i = 0; set != NULL && i < set->count; ++i) {
        const did_poll_entry_t *e = &set->entries[i];
        if (e->state != DID_POLL_SUPPORTED && e->state != DID_POLL_NOT_READY) continue;
        if (e->next_due_us > now_us) continue;
        if (best < 0 || e->next_due_us < best_due) {
            best = (int)i;
            best_due = e->next_due_us;
        }
    }
    return best;
}

size_t did_poll_count(const did_poll_set_t *set, did_poll_state_t state)
{
    size_t n = 0U;
    for (size_t i = 0; set != NULL && i < set->count; ++i) {
        if (set->entries[i].state == state) ++n;
    }
    return n;
}

bool did_poll_nrc_means_unsupported(uint8_t nrc)
{
    switch (nrc) {
    case 0x21U: /* busy */
    case 0x22U: /* conditionsNotCorrect */
    case 0x24U:
    case 0x25U:
    case 0x26U:
    case 0x78U: /* responsePending */
        return false;
    default:
        return true; /* includes 0x31 requestOutOfRange */
    }
}

void did_poll_record_positive(did_poll_set_t *set, size_t index, int64_t now_us)
{
    if (set == NULL || index >= set->count) return;
    did_poll_entry_t *e = &set->entries[index];
    e->state = DID_POLL_SUPPORTED;
    e->timeouts = 0U;
    e->next_due_us = now_us + (int64_t)e->interval_ms * 1000LL;
}

void did_poll_record_pending(did_poll_set_t *set, size_t index, int64_t now_us)
{
    if (set == NULL || index >= set->count) return;
    did_poll_entry_t *e = &set->entries[index];
    if (e->state == DID_POLL_EXCLUDED) return;
    e->state = DID_POLL_NOT_READY;
    e->next_due_us = now_us + (int64_t)DID_POLL_NOT_READY_MS * 1000LL;
}

void did_poll_record_negative(did_poll_set_t *set, size_t index, uint8_t nrc,
                              int64_t now_us)
{
    if (set == NULL || index >= set->count) return;
    did_poll_entry_t *e = &set->entries[index];
    e->last_nrc = nrc;
    if (did_poll_nrc_means_unsupported(nrc)) {
        e->state = DID_POLL_EXCLUDED;
    } else {
        did_poll_record_pending(set, index, now_us);
    }
}

void did_poll_record_timeout(did_poll_set_t *set, size_t index, int64_t now_us)
{
    if (set == NULL || index >= set->count) return;
    did_poll_entry_t *e = &set->entries[index];
    if (e->state == DID_POLL_NOT_READY) {
        /* A DID that already answered "pending" stays supported; try again later. */
        e->next_due_us = now_us + (int64_t)DID_POLL_NOT_READY_MS * 1000LL;
        return;
    }
    if (e->state == DID_POLL_SUPPORTED) {
        /* A DID the ECU already answered is never dropped for silence (ignition off,
         * bus glitch): back off and try again. */
        e->next_due_us = now_us + (int64_t)DID_POLL_SILENT_BACKOFF_MS * 1000LL;
        return;
    }
    if (++e->timeouts >= DID_POLL_MAX_TIMEOUTS) {
        e->state = DID_POLL_EXCLUDED;
    }
}
