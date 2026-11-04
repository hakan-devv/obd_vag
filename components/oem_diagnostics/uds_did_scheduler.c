#include "uds_did_scheduler.h"

#include <string.h>

static bool item_matches(const uds_did_scheduler_item_t *item,
                         uint16_t did,
                         diag_ecu_role_t role)
{
    return item != NULL && item->armed && item->did == did && item->role == role;
}

void uds_did_scheduler_init(uds_did_scheduler_t *scheduler)
{
    if (scheduler == NULL) {
        return;
    }
    memset(scheduler, 0, sizeof(*scheduler));
    scheduler->item_count = 0U;
    scheduler->now_ms = 0U;
}

bool uds_did_scheduler_can_enqueue(const uds_did_descriptor_t *descriptor,
                                 const char *profile_id,
                                 bool session_ready,
                                 bool allow_experimental)
{
    if (descriptor == NULL) {
        return false;
    }
    if (!descriptor->auto_query_allowed) {
        return false;
    }
    if (!allow_experimental && descriptor->validation < UDS_DID_VALIDATION_SOURCE_BACKED) {
        return false;
    }
    if (descriptor->session_required && !session_ready) {
        return false;
    }
    if (profile_id != NULL && profile_id[0] != '\0' &&
        descriptor->profile_id != NULL && descriptor->profile_id[0] != '\0' &&
        strcmp(descriptor->profile_id, profile_id) != 0) {
        return false;
    }
    return true;
}

bool uds_did_scheduler_enqueue(uds_did_scheduler_t *scheduler,
                              const uds_did_descriptor_t *descriptor,
                              uint32_t now_ms,
                              bool allow_experimental)
{
    if (scheduler == NULL || descriptor == NULL) {
        return false;
    }
    if (!uds_did_scheduler_can_enqueue(descriptor, NULL, true, allow_experimental)) {
        return false;
    }
    if (scheduler->item_count >= (sizeof(scheduler->items) / sizeof(scheduler->items[0]))) {
        return false;
    }
    for (size_t i = 0; i < scheduler->item_count; ++i) {
        if (item_matches(&scheduler->items[i], descriptor->did, descriptor->role)) {
            return false;
        }
    }
    scheduler->items[scheduler->item_count].did = descriptor->did;
    scheduler->items[scheduler->item_count].role = descriptor->role;
    scheduler->items[scheduler->item_count].next_due_ms = now_ms + descriptor->poll_interval_ms;
    scheduler->items[scheduler->item_count].armed = true;
    scheduler->item_count += 1U;
    scheduler->now_ms = now_ms;
    return true;
}

bool uds_did_scheduler_take_next(uds_did_scheduler_t *scheduler,
                                uint32_t now_ms,
                                uint16_t *did,
                                diag_ecu_role_t *role)
{
    if (scheduler == NULL || did == NULL || role == NULL) {
        return false;
    }
    size_t chosen_index = SIZE_MAX;
    uint32_t selected_due = UINT32_MAX;
    for (size_t i = 0; i < scheduler->item_count; ++i) {
        uds_did_scheduler_item_t *item = &scheduler->items[i];
        if (!item->armed) continue;
        if (item->next_due_ms <= now_ms && item->next_due_ms < selected_due) {
            selected_due = item->next_due_ms;
            chosen_index = i;
        }
    }
    if (chosen_index == SIZE_MAX) {
        return false;
    }
    uds_did_scheduler_item_t *selected = &scheduler->items[chosen_index];
    *did = selected->did;
    *role = selected->role;
    selected->next_due_ms = now_ms + 1000U;
    return true;
}
