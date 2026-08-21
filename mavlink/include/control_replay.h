#ifndef CONTROL_REPLAY_H
#define CONTROL_REPLAY_H

#include <stdbool.h>

#include "replay.h"

bool prepare_replay_trajectory(
    const char *path,
    ReplayTrajectory_t *resampled_trajectory
);

#endif
