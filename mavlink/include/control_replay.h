#ifndef CONTROL_REPLAY_H
#define CONTROL_REPLAY_H

#include <stdbool.h>

#include "replay.h"

bool prepare_replay_trajectory(
    const char *path,
    ReplayTrajectory_t *resampled_trajectory
);

bool control_replay_apply_origin(
    ReplayTrajectory_t *trajectory,
    float origin_x,
    float origin_y,
    float origin_z
);

#endif
