#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>

#include "../../include/flight_data.h"
#include "../../include/tlog.h"
#include "../../include/replay.h"

int main(int argc, char* argv[]) {
    int exit_status = 0;

    /* ---------- open the file ---------- */
    if (argc != 2) {
        fprintf(stderr, "usage: %s <telemetry.tlog>\n", argv[0]);
        return 1;
    }

    FILE *file = fopen(argv[1], "rb");

    if (file == NULL) {
        fprintf(stderr, "cannot open %s: %s\n", argv[1], strerror(errno));
        return 1;
    }

    /* ---------- load flight data ---------- */
    FlightData_t flight_data = {0};
    TLogLoadStats_t stats = {0};

    printf("READING %s\n", argv[1]);

    if (!tlog_load_flight_data(file, &flight_data, &stats)) {
        fprintf(stderr, "failed to load tlog\n");
        exit_status = 1;
    }

    // print_read_summary(&flight_data, &stats);


    /* ---------- replay! ---------- */

    // check if initial timestamp exist
    if (flight_data.local_position_count == 0) {
        fprintf(stderr, "ERROR READING LOCAL_POSITION_COUNT: local_position_count = 0\n");
        flight_data_free(&flight_data);

        if (fclose(file) == EOF) {
            perror("fclose");
        }

        return 1;
    }

    ReplayTrajectory_t trajectory = {0};
    if (!replay_trajectory_from_flight_data(&flight_data, &trajectory)) {
        return 1;
    };

    ReplayTrajectory_t resample = {0};
    if (!replay_trajectory_resample(&trajectory, 100000, &resample)) {
        return 1;
    };

    TrajectorySafetyLimit_t limits = {
        .maximum_altitude = 3.0f,
        .maximum_horizontal_distance = 0.5f,
        .speed = 1.0f,
        .total_duration_us = UINT64_C(25000000) 
    };

    if (!replay_trajectory_validate(&resample, &limits)) {
        return 1;
    }
/*
    TrajectorySafetyLimit_t limits_physical = {
        .maximum_altitude = 1.5f,
        .maximum_horizontal_distance = 1.0f,
        .speed = 0.5f,
        .total_duration_us = UINT64_C(15000000)
    };
*/

    for (size_t i = 0; i < resample.count; i++) {
        printf("%8.3" PRIu64 " us | x=%+.6f y=%+.6f z=%+.6f\n", 
                resample.positions[i].elapsed_us, 
                resample.positions[i].x, 
                resample.positions[i].y, 
                resample.positions[i].z);
    }

    /* ---------- close ---------- */
    flight_data_free(&flight_data);
    replay_trajectory_free(&trajectory);
    replay_trajectory_free(&resample);

    /* close the file */
    if (fclose(file) == EOF) {
        perror("fclose");
        return 1;
    }

    return exit_status;
}
