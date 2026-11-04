#ifndef UDS_DID_SCHEDULER_H
#define UDS_DID_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#include "diag_ecu_profile.h"
#include "uds_did_catalog.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t did;
    diag_ecu_role_t role;
    uint32_t next_due_ms;
    bool armed;
} uds_did_scheduler_item_t;

typedef struct {
    uds_did_scheduler_item_t items[8];
    size_t item_count;
    uint32_t now_ms;
} uds_did_scheduler_t;

void uds_did_scheduler_init(uds_did_scheduler_t *scheduler);
bool uds_did_scheduler_can_enqueue(const uds_did_descriptor_t *descriptor,
                                 const char *profile_id,
                                 bool session_ready,
                                 bool allow_experimental);
bool uds_did_scheduler_enqueue(uds_did_scheduler_t *scheduler,
                              const uds_did_descriptor_t *descriptor,
                              uint32_t now_ms,
                              bool allow_experimental);
bool uds_did_scheduler_take_next(uds_did_scheduler_t *scheduler,
                                uint32_t now_ms,
                                uint16_t *did,
                                diag_ecu_role_t *role);

#ifdef __cplusplus
}
#endif

#endif
