#ifndef REPLAY_H
#define REPLAY_H

#include <stddef.h>
#include <stdint.h>

#include "flight_data.h"

typedef struct {
    uint64_t elapsed_us;
    float x;
    float y;
    float z;
} ReplayPosition_t;

typedef struct {
    ReplayPosition_t *positions;
    size_t count;
} ReplayTrajectory_t;

typedef struct {
    float maximum_altitude;
    float maximum_horizontal_distance;
    float speed;
    uint64_t total_duration_us;
} TrajectorySafetyLimit_t;

bool replay_trajectory_from_flight_data(const FlightData_t *flight_data, ReplayTrajectory_t *trajectory);
void replay_trajectory_free(ReplayTrajectory_t *trajectory);
bool replay_trajectory_resample(const ReplayTrajectory_t *trajectory, uint64_t interval_us, ReplayTrajectory_t *output);

bool replay_trajectory_validate(const ReplayTrajectory_t *trajectory, TrajectorySafetyLimit_t *limits);

#endif
