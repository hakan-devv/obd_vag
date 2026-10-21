#include "pid_scheduler.h"

uint8_t pid_scheduler_select(const parameter_descriptor_t *catalog, size_t count,
                             const bool supported[256],
                             const int64_t next_due_us[256], int64_t now_us)
{
    if (catalog == NULL || supported == NULL || next_due_us == NULL) return 0xFFU;

    const parameter_descriptor_t *selected = NULL;
    for (size_t i = 0; i < count; ++i) {
        const parameter_descriptor_t *candidate = &catalog[i];
        const uint8_t pid = candidate->pid;
        if (!supported[pid] || (next_due_us[pid] != 0 &&
                                now_us < next_due_us[pid])) continue;
        if (selected == NULL ||
            next_due_us[pid] < next_due_us[selected->pid] ||
            (next_due_us[pid] == next_due_us[selected->pid] &&
             candidate->priority < selected->priority)) {
            selected = candidate;
        }
    }
    return selected != NULL ? selected->pid : 0xFFU;
}
