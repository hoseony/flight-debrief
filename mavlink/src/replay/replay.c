#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../../include/replay.h"
#include "../../include/flight_data.h"

bool replay_trajectory_from_flight_data(const FlightData_t *flight_data, ReplayTrajectory_t *trajectory) {
    if (flight_data == NULL ||trajectory == NULL) {
        return false;
    }

    *trajectory = (ReplayTrajectory_t){0};

    if (flight_data->local_position_count == 0) {
        return false;
    }

    // allocate memory... :(
    trajectory->positions = malloc(flight_data->local_position_count * sizeof(*trajectory->positions));

    // check malloc failure
    if (trajectory->positions == NULL) {
        return false;
    }

    // 
    uint64_t initial_timestamp = flight_data->local_positions[0].timestamp_us;
    float initial_x = flight_data->local_positions[0].local_position.x;
    float initial_y = flight_data->local_positions[0].local_position.y;
    float initial_z = flight_data->local_positions[0].local_position.z;

    for (size_t i = 0; i < flight_data->local_position_count; i ++) {
        trajectory->positions[i].elapsed_us = flight_data->local_positions[i].timestamp_us - initial_timestamp;
        
        float x = flight_data->local_positions[i].local_position.x - initial_x;
        float y = flight_data->local_positions[i].local_position.y - initial_y;
        float z = flight_data->local_positions[i].local_position.z - initial_z;

        // we can put some safety related stuff here

        trajectory->positions[i].x = x;
        trajectory->positions[i].y = y;
        trajectory->positions[i].z = z;
    }

    trajectory->count = flight_data->local_position_count;

    return true;
}

void replay_trajectory_free(ReplayTrajectory_t *trajectory) {
    if (trajectory == NULL) {
        return;
    }

    free(trajectory->positions);
    *trajectory = (ReplayTrajectory_t){0};
}

/// this will resample the trajectory based on the interval.
/// It will use linear interpolation to find the position.
bool replay_trajectory_resample(const ReplayTrajectory_t *trajectory, uint64_t interval_us, ReplayTrajectory_t *output) {
    if (trajectory == NULL || interval_us == 0 || output == NULL) {
        return false;
    }

    *output = (ReplayTrajectory_t){0};

    uint64_t last_t = trajectory->positions[trajectory->count - 1].elapsed_us;
    size_t output_count = last_t / interval_us + 1;

    output->positions = malloc(output_count * sizeof(*output->positions));

    if (output->positions == NULL) {
        return false;
    }

    size_t upper = 0;

    for (size_t i = 0; i < output_count; i++) {
        uint64_t target_us = i * interval_us;

        output->positions[i].elapsed_us = target_us;

        // find the index right of upper (index of sample that is right higher than the target)
        while (upper < trajectory->count && trajectory->positions[upper].elapsed_us < target_us) {
            upper++;
        }

        if (trajectory->positions[upper].elapsed_us == target_us) {
            // already have exact timestamp
            output->positions[i].x = trajectory->positions[upper].x;
            output->positions[i].y = trajectory->positions[upper].y;
            output->positions[i].z = trajectory->positions[upper].z;
        } else {
            // linear interpolation
            
            size_t lower = upper -1;

            ReplayPosition_t *a = &trajectory->positions[lower];
            ReplayPosition_t *b = &trajectory->positions[upper];

            // how far is the sample between the sample?
            double t = (double)(target_us - a->elapsed_us) / (double)(b->elapsed_us - a->elapsed_us);

            // x(t) = p0 + t * (p1 - p0)
            output->positions[i].x = a->x + t * (b->x - a->x);
            output->positions[i].y = a->y + t * (b->y - a->y);
            output->positions[i].z = a->z + t * (b->z - a->z);
        }
    }

    output->count = output_count;
    return true;
}

/// This must be ran before actual testing
/// It checks the safety limit, empty trajectory, NAN/Inf, sudden position jump
bool replay_trajectory_validate(const ReplayTrajectory_t *trajectory, TrajectorySafetyLimit_t *limits) {
    if (trajectory == NULL ||trajectory->positions == NULL || limits == NULL) {
        fprintf(stderr, "NULL: check input parameters");
        return false;
    }

    if (trajectory->count == 0) {
        fprintf(stderr, "DATA: trajectory->count should not be 0");
        return false;
    }

    if (trajectory->positions[trajectory->count - 1].elapsed_us >= limits->total_duration_us) {
        return false;
    }

    for (size_t i = 0; i < trajectory->count; i++) {
        ReplayPosition_t *position = &trajectory->positions[i];

        // check if finite
        if (!isfinite(position->x) || !isfinite(position->y) || !isfinite(position->z)) {
            fprintf(stderr, "sample %zu rejected: non-finite position\n", i);
            return false;
        }

        // check max altitude, horizontal distance
        float altitude = -position->z;
        float horizontal_distance = hypotf(position->x, position->y);

        if (altitude > limits->maximum_altitude) {
            fprintf(stderr, "sample %zu rejected: trajectory out of maximum_altitude", i);
            return false;
        }

        if (horizontal_distance > limits->maximum_horizontal_distance) {
            fprintf(stderr, "sample %zu rejected: trajectory out of maximum_horizontal_distance", i);
            return false;
        }

        if (i >= 1) {
            // check time
            // if somehow time is weird, not safe 
            ReplayPosition_t *current= &trajectory->positions[i];
            ReplayPosition_t *previous = &trajectory->positions[i - 1];

            if (current->elapsed_us <= previous->elapsed_us) {
                return false;
            }
            
            // check speed
            uint64_t dt_us = current->elapsed_us - previous->elapsed_us;

            float dx = current->x - previous->x;
            float dy = current->y - previous->y;
            float dz = current->z - previous->z; 
            
            float distance = sqrtf(dx*dx + dy*dy +dz*dz);
            float dt_seconds = (float)dt_us / 1000000.0f;
            float speed = distance / dt_seconds;

            if (speed > limits->speed) {
                fprintf(stderr, "sample %zu rejected: trajectory exceeded maximum_speed", i);
                return false;
            }
        } 
    }

    return true;
}
