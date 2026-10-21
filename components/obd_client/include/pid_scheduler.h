#ifndef PID_SCHEDULER_H
#define PID_SCHEDULER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "parameter_manager.h"

uint8_t pid_scheduler_select(const parameter_descriptor_t *catalog, size_t count,
                             const bool supported[256],
                             const int64_t next_due_us[256], int64_t now_us);

#endif
