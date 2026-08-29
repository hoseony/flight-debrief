#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../include/control_replay.h"
#include "../../include/flight_data.h"
#include "../../include/tlog.h"

#define COLOR_RED "\033[31m"
#define COLOR_RESET "\033[0m"
#define LOG_ERROR COLOR_RED "[ERROR]" COLOR_RESET

bool prepare_replay_trajectory(
        const char *path,
        ReplayTrajectory_t *resampled_trajectory) {
    if (path == NULL || resampled_trajectory == NULL) {
        return false;
    }

    *resampled_trajectory = (ReplayTrajectory_t){0};

    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        fprintf(stderr, LOG_ERROR " Failed to open log '%s': %s\n",
                path, strerror(errno));
        return false;
    }

    FlightData_t flight_data = {0};
    ReplayTrajectory_t trajectory = {0};
    TLogLoadStats_t stats = {0};
    bool success = false;

    printf("[INFO] Loading trajectory: %s\n", path);

    success = tlog_load_flight_data(file, &flight_data, &stats);
    if (!success) {
        fprintf(stderr, LOG_ERROR " Failed to load tlog\n");
    }

    if (success && flight_data.local_position_count == 0) {
        fprintf(stderr, LOG_ERROR " Trajectory contains no local-position samples\n");
        success = false;
    }

    if (success) {
        success = replay_trajectory_from_flight_data(&flight_data, &trajectory);
    }

    if (success) {
        success = replay_trajectory_resample(
                &trajectory, UINT64_C(100000), resampled_trajectory);
    }

    // Internal limit
    TrajectorySafetyLimit_t limits = {
        .maximum_altitude = 3.0f,
        .maximum_horizontal_distance = 0.5f,
        .speed = 1.0f,
        .total_duration_us = UINT64_C(30000000)
    };

    if (success) {
        success = replay_trajectory_validate(resampled_trajectory, &limits);
    }

    if (success) {
        printf("[INFO] Trajectory ready: samples=%zu\n",
                resampled_trajectory->count);
    }

    flight_data_free(&flight_data);
    replay_trajectory_free(&trajectory);

    if (fclose(file) == EOF) {
        perror(LOG_ERROR " Failed to close telemetry log");
        success = false;
    }

    if (!success) {
        replay_trajectory_free(resampled_trajectory);
    }

    return success;
}

bool control_replay_apply_origin(
        ReplayTrajectory_t *trajectory,
        float origin_x,
        float origin_y,
        float origin_z) {
    if (trajectory == NULL
            || trajectory->positions == NULL
            || trajectory->count == 0
            || !isfinite(origin_x)
            || !isfinite(origin_y)
            || !isfinite(origin_z)) {
        return false;
    }

    for (size_t i = 0; i < trajectory->count; i++) {
        const ReplayPosition_t *position = &trajectory->positions[i];

        if (!isfinite(position->x + origin_x)
                || !isfinite(position->y + origin_y)
                || !isfinite(position->z + origin_z)) {
            return false;
        }
    }

    for (size_t i = 0; i < trajectory->count; i++) {
        ReplayPosition_t *position = &trajectory->positions[i];
        position->x += origin_x;
        position->y += origin_y;
        position->z += origin_z;
    }

    return true;
}
