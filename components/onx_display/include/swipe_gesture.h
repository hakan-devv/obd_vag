#ifndef SWIPE_GESTURE_H
#define SWIPE_GESTURE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t start_x;
    int16_t start_y;
    int16_t last_x;
    int16_t last_y;
    bool active;
} swipe_gesture_t;

typedef enum {
    SWIPE_NONE = 0,
    SWIPE_LEFT,
    SWIPE_RIGHT,
} swipe_direction_t;

void swipe_gesture_begin(swipe_gesture_t *gesture, int16_t x, int16_t y);
void swipe_gesture_update(swipe_gesture_t *gesture, int16_t x, int16_t y);
swipe_direction_t swipe_gesture_end(swipe_gesture_t *gesture, int16_t x, int16_t y);

#endif
