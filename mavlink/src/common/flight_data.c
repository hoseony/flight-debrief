#include <stdlib.h>

#include "../../include/mavlink_types.h"
#include "../../include/flight_data.h"

bool flight_data_add_heartbeat(FlightData_t *data, uint64_t timestamp_us, const MAVLinkHeartbeat_t *heartbeat) {
    if (data == NULL || heartbeat == NULL) {
        return false;
    }

    if (data->heartbeat_count == data->heartbeat_capacity) {
        size_t new_capacity = data->heartbeat_capacity == 0 ? 256 : data->heartbeat_capacity * 2;
        HeartbeatSample_t *new_samples = realloc(data->heartbeats, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->heartbeats = new_samples;
        data->heartbeat_capacity = new_capacity;
    }

    HeartbeatSample_t *sample = &data->heartbeats[data->heartbeat_count];
    sample->timestamp_us = timestamp_us;
    sample->heartbeat = *heartbeat;
    data->heartbeat_count++;
    return true;
}

bool flight_data_add_attitude(FlightData_t *data, uint64_t timestamp_us, const MAVLinkAttitude_t *attitude) {
    if (data == NULL || attitude == NULL) {
        return false;
    }

    if (data->attitude_count == data->attitude_capacity) {
        size_t new_capacity = data->attitude_capacity == 0 ? 256 : data->attitude_capacity * 2;
        AttitudeSample_t *new_samples = realloc(data->attitudes, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->attitudes = new_samples;
        data->attitude_capacity = new_capacity;
    }

    AttitudeSample_t *sample = &data->attitudes[data->attitude_count];
    sample->timestamp_us = timestamp_us;
    sample->attitude = *attitude;
    data->attitude_count++;
    return true;
}

bool flight_data_add_attitude_quaternion(FlightData_t *data, uint64_t timestamp_us, const MAVLinkAttitudeQuaternion_t *attitude) {
    if (data == NULL || attitude == NULL) {
        return false;
    }

    if (data->attitude_quaternion_count == data->attitude_quaternion_capacity) {
        size_t new_capacity = data->attitude_quaternion_capacity == 0 ? 256 : data->attitude_quaternion_capacity * 2;
        AttitudeQuaternionSample_t *new_samples = realloc(data->attitude_quaternions, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->attitude_quaternions = new_samples;
        data->attitude_quaternion_capacity = new_capacity;
    }

    AttitudeQuaternionSample_t *sample = &data->attitude_quaternions[data->attitude_quaternion_count];
    sample->timestamp_us = timestamp_us;
    sample->attitude_quaternion = *attitude;
    data->attitude_quaternion_count++;
    return true;
}

bool flight_data_add_local_position(FlightData_t *data, uint64_t timestamp_us, const MAVLinkLocalPositionNed_t *position) {
    if (data == NULL || position == NULL) {
        return false;
    }

    if (data->local_position_count == data->local_position_capacity) {
        size_t new_capacity = data->local_position_capacity == 0 ? 256 : data->local_position_capacity * 2;
        LocalPositionNedSample_t *new_samples = realloc(data->local_positions, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->local_positions = new_samples;
        data->local_position_capacity = new_capacity;
    }

    LocalPositionNedSample_t *sample = &data->local_positions[data->local_position_count];
    sample->timestamp_us = timestamp_us;
    sample->local_position = *position;
    data->local_position_count++;
    return true;
}

bool flight_data_add_global_position(FlightData_t *data, uint64_t timestamp_us, const MAVLinkGlobalPositionInt_t *position) {
    if (data == NULL || position == NULL) {
        return false;
    }

    if (data->global_position_count == data->global_position_capacity) {
        size_t new_capacity = data->global_position_capacity == 0 ? 256 : data->global_position_capacity * 2;
        GlobalPositionIntSample_t *new_samples = realloc(data->global_positions, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->global_positions = new_samples;
        data->global_position_capacity = new_capacity;
    }

    GlobalPositionIntSample_t *sample = &data->global_positions[data->global_position_count];
    sample->timestamp_us = timestamp_us;
    sample->global_position = *position;
    data->global_position_count++;
    return true;
}

bool flight_data_add_servo_output(FlightData_t *data, uint64_t timestamp_us, const MAVLinkServoOutputRaw_t *servo_output) {
    if (data == NULL || servo_output == NULL) {
        return false;
    }

    if (data->servo_output_count == data->servo_output_capacity) {
        size_t new_capacity = data->servo_output_capacity == 0 ? 256 : data->servo_output_capacity * 2;
        ServoOutputRawSample_t *new_samples = realloc(data->servo_outputs, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->servo_outputs = new_samples;
        data->servo_output_capacity = new_capacity;
    }

    ServoOutputRawSample_t *sample = &data->servo_outputs[data->servo_output_count];
    sample->timestamp_us = timestamp_us;
    sample->servo_output = *servo_output;
    data->servo_output_count++;
    return true;
}

bool flight_data_add_attitude_target(FlightData_t *data, uint64_t timestamp_us, const MAVLinkAttitudeTarget_t *attitude_target) {
    if (data == NULL || attitude_target == NULL) {
        return false;
    }

    if (data->attitude_target_count == data->attitude_target_capacity) {
        size_t new_capacity = data->attitude_target_capacity == 0 ? 256 : data->attitude_target_capacity * 2;
        AttitudeTargetSample_t *new_samples = realloc(data->attitude_targets, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->attitude_targets = new_samples;
        data->attitude_target_capacity = new_capacity;
    }

    AttitudeTargetSample_t *sample = &data->attitude_targets[data->attitude_target_count];
    sample->timestamp_us = timestamp_us;
    sample->attitude_target = *attitude_target;
    data->attitude_target_count++;
    return true;
}

bool flight_data_add_set_position_target(FlightData_t *data, uint64_t timestamp_us, const MAVLinkSetPositionTargetLocalNed_t *target) {
    if (data == NULL || target == NULL) {
        return false;
    }

    if (data->set_position_target_count == data->set_position_target_capacity) {
        size_t new_capacity = data->set_position_target_capacity == 0 ? 256 : data->set_position_target_capacity * 2;
        SetPositionTargetLocalNedSample_t *new_samples = realloc(data->set_position_targets, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->set_position_targets = new_samples;
        data->set_position_target_capacity = new_capacity;
    }

    SetPositionTargetLocalNedSample_t *sample = &data->set_position_targets[data->set_position_target_count];
    sample->timestamp_us = timestamp_us;
    sample->set_position_target = *target;
    data->set_position_target_count++;
    return true;
}

bool flight_data_add_position_target(FlightData_t *data, uint64_t timestamp_us, const MAVLinkPositionTargetLocalNed_t *target) {
    if (data == NULL || target == NULL) {
        return false;
    }

    if (data->position_target_count == data->position_target_capacity) {
        size_t new_capacity = data->position_target_capacity == 0 ? 256 : data->position_target_capacity * 2;
        PositionTargetLocalNedSample_t *new_samples = realloc(data->position_targets, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        data->position_targets = new_samples;
        data->position_target_capacity = new_capacity;
    }

    PositionTargetLocalNedSample_t *sample = &data->position_targets[data->position_target_count];
    sample->timestamp_us = timestamp_us;
    sample->position_target = *target;
    data->position_target_count++;
    return true;
}

void flight_data_free(FlightData_t *data) {
    if (data == NULL) {
        return;
    }

    free(data->heartbeats);
    free(data->attitudes);
    free(data->attitude_quaternions);
    free(data->local_positions);
    free(data->global_positions);
    free(data->servo_outputs);
    free(data->attitude_targets);
    free(data->set_position_targets);
    free(data->position_targets);

    *data = (FlightData_t){0};
}
