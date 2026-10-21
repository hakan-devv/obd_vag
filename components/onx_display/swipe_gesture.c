#include "swipe_gesture.h"

#include <stddef.h>

#define SWIPE_MIN_DISTANCE 45
#define SWIPE_AXIS_DOMINANCE 2

void swipe_gesture_begin(swipe_gesture_t *gesture, int16_t x, int16_t y)
{
    if (gesture == NULL) return;
    *gesture = (swipe_gesture_t) {
        .start_x = x,
        .start_y = y,
        .last_x = x,
        .last_y = y,
        .active = true,
    };
}

void swipe_gesture_update(swipe_gesture_t *gesture, int16_t x, int16_t y)
{
    if (gesture == NULL || !gesture->active) return;
    gesture->last_x = x;
    gesture->last_y = y;
}

swipe_direction_t swipe_gesture_end(swipe_gesture_t *gesture, int16_t x, int16_t y)
{
    if (gesture == NULL || !gesture->active) return SWIPE_NONE;
    swipe_gesture_update(gesture, x, y);
    gesture->active = false;

    const int16_t dx = (int16_t)(gesture->last_x - gesture->start_x);
    const int16_t dy = (int16_t)(gesture->last_y - gesture->start_y);
    const int32_t abs_dx = dx < 0 ? -(int32_t)dx : dx;
    const int32_t abs_dy = dy < 0 ? -(int32_t)dy : dy;
    if (abs_dx < SWIPE_MIN_DISTANCE || abs_dx < abs_dy * SWIPE_AXIS_DOMINANCE) {
        return SWIPE_NONE;
    }
    return dx < 0 ? SWIPE_LEFT : SWIPE_RIGHT;
}
