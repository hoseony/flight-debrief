#include <errno.h>
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
