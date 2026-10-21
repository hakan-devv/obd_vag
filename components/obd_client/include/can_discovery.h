#ifndef CAN_DISCOVERY_H
#define CAN_DISCOVERY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_driver.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_DISCOVERY_MIN_VALID_FRAMES 3U
#define CAN_DISCOVERY_CANDIDATE_COUNT  4U

typedef enum {
    CAN_DISCOVERY_LISTENING = 0,
    CAN_DISCOVERY_VERIFYING,
    CAN_DISCOVERY_LOCKED,
    CAN_DISCOVERY_EXHAUSTED,
} can_discovery_phase_t;

typedef struct {
    can_bus_config_t candidates[CAN_DISCOVERY_CANDIDATE_COUNT];
    size_t candidate_index;
    uint8_t matching_frames;
    can_discovery_phase_t phase;
} can_discovery_t;

void can_discovery_init(can_discovery_t *discovery,
                        const can_bus_config_t *first_candidate);
const can_bus_config_t *can_discovery_candidate(const can_discovery_t *discovery);
bool can_discovery_observe(can_discovery_t *discovery, const can_frame_t *frame);
bool can_discovery_has_evidence(const can_discovery_t *discovery);
esp_err_t can_discovery_begin_verification(can_discovery_t *discovery);
bool can_discovery_confirm(can_discovery_t *discovery, const can_frame_t *frame,
                           const uint8_t *payload, size_t length);
esp_err_t can_discovery_lock_passive(can_discovery_t *discovery);
void can_discovery_reject_candidate(can_discovery_t *discovery);
void can_discovery_restart(can_discovery_t *discovery);

#ifdef __cplusplus
}
#endif

#endif
