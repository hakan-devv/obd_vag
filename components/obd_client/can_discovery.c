#include "can_discovery.h"

#include <string.h>

static bool same_config(const can_bus_config_t *left, const can_bus_config_t *right)
{
    return left->bitrate == right->bitrate &&
           left->extended_ids == right->extended_ids;
}

void can_discovery_init(can_discovery_t *discovery,
                        const can_bus_config_t *first_candidate)
{
    if (discovery == NULL || first_candidate == NULL) return;

    static const can_bus_config_t all_candidates[] = {
        {CAN_BITRATE_250_KBPS, false},
        {CAN_BITRATE_250_KBPS, true},
        {CAN_BITRATE_500_KBPS, false},
        {CAN_BITRATE_500_KBPS, true},
    };

    memset(discovery, 0, sizeof(*discovery));
    discovery->candidates[0] = *first_candidate;
    size_t output_index = 1U;
    for (unsigned priority = 0; priority < 3U; ++priority) {
        for (size_t i = 0; i < sizeof(all_candidates) / sizeof(all_candidates[0]); ++i) {
            const can_bus_config_t *candidate = &all_candidates[i];
            if (same_config(first_candidate, candidate)) continue;

            const unsigned candidate_priority =
                candidate->bitrate == first_candidate->bitrate ? 0U :
                candidate->extended_ids == first_candidate->extended_ids ? 1U : 2U;
            if (candidate_priority == priority) {
                discovery->candidates[output_index++] = *candidate;
            }
        }
    }
    discovery->phase = CAN_DISCOVERY_LISTENING;
}

const can_bus_config_t *can_discovery_candidate(const can_discovery_t *discovery)
{
    if (discovery == NULL ||
        discovery->candidate_index >= CAN_DISCOVERY_CANDIDATE_COUNT ||
        discovery->phase == CAN_DISCOVERY_EXHAUSTED) {
        return NULL;
    }
    return &discovery->candidates[discovery->candidate_index];
}

bool can_discovery_observe(can_discovery_t *discovery, const can_frame_t *frame)
{
    if (discovery == NULL || frame == NULL ||
        discovery->phase != CAN_DISCOVERY_LISTENING ||
        frame->rtr || frame->dlc > sizeof(frame->data)) {
        return false;
    }
    const can_bus_config_t *candidate = can_discovery_candidate(discovery);
    if (candidate == NULL || frame->extended != candidate->extended_ids) return false;
    if (discovery->matching_frames < CAN_DISCOVERY_MIN_VALID_FRAMES) {
        ++discovery->matching_frames;
    }
    return true;
}

bool can_discovery_has_evidence(const can_discovery_t *discovery)
{
    return discovery != NULL &&
           discovery->phase == CAN_DISCOVERY_LISTENING &&
           discovery->matching_frames >= CAN_DISCOVERY_MIN_VALID_FRAMES;
}

esp_err_t can_discovery_begin_verification(can_discovery_t *discovery)
{
    if (!can_discovery_has_evidence(discovery)) return ESP_ERR_INVALID_STATE;
    discovery->phase = CAN_DISCOVERY_VERIFYING;
    return ESP_OK;
}

bool can_discovery_confirm(can_discovery_t *discovery, const can_frame_t *frame,
                           const uint8_t *payload, size_t length)
{
    if (discovery == NULL || frame == NULL || payload == NULL ||
        discovery->phase != CAN_DISCOVERY_VERIFYING || length != 6U ||
        payload[0] != 0x41U || payload[1] != 0x00U ||
        frame->rtr || frame->dlc > sizeof(frame->data)) {
        return false;
    }

    const can_bus_config_t *candidate = can_discovery_candidate(discovery);
    if (candidate == NULL || frame->extended != candidate->extended_ids) return false;
    if (frame->extended) {
        if (frame->id < 0x18DAF100U || frame->id > 0x18DAF1FFU) return false;
    } else if (frame->id < 0x7E8U || frame->id > 0x7EFU) {
        return false;
    }

    discovery->phase = CAN_DISCOVERY_LOCKED;
    return true;
}

esp_err_t can_discovery_lock_passive(can_discovery_t *discovery)
{
    if (!can_discovery_has_evidence(discovery)) return ESP_ERR_INVALID_STATE;
    discovery->phase = CAN_DISCOVERY_LOCKED;
    return ESP_OK;
}

void can_discovery_reject_candidate(can_discovery_t *discovery)
{
    if (discovery == NULL ||
        (discovery->phase != CAN_DISCOVERY_VERIFYING &&
         discovery->phase != CAN_DISCOVERY_LISTENING)) {
        return;
    }
    ++discovery->candidate_index;
    discovery->matching_frames = 0U;
    discovery->phase = discovery->candidate_index < CAN_DISCOVERY_CANDIDATE_COUNT
        ? CAN_DISCOVERY_LISTENING : CAN_DISCOVERY_EXHAUSTED;
}

void can_discovery_restart(can_discovery_t *discovery)
{
    if (discovery == NULL || discovery->phase != CAN_DISCOVERY_EXHAUSTED) return;
    discovery->candidate_index = 0U;
    discovery->matching_frames = 0U;
    discovery->phase = CAN_DISCOVERY_LISTENING;
}
