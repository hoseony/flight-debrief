#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

#include <stdint.h>
#include "mavlink_types.h"

typedef struct {
    uint64_t timestamp_us;
    MAVLinkHeartbeat_t heartbeat;
} HeartbeatSample_t;

// ATTITUDE: sampled data point
// this should be dynamically allocated
typedef struct {
    uint64_t timestamp_us;
    MAVLinkAttitude_t attitude;
} AttitudeSample_t;

typedef struct {
    uint64_t timestamp_us;
    MAVLinkAttitudeQuaternion_t attitude_quaternion;
} AttitudeQuaternionSample_t;

typedef struct {
    uint64_t timestamp_us;
    MAVLinkLocalPositionNed_t local_position;
} LocalPositionNedSample_t;

typedef struct {
    uint64_t timestamp_us;
    MAVLinkGlobalPositionInt_t global_position;
} GlobalPositionIntSample_t;

typedef struct {
    uint64_t timestamp_us;
    MAVLinkSetPositionTargetLocalNed_t set_position_target;
} SetPositionTargetLocalNedSample_t;

typedef struct {
    uint64_t timestamp_us;
    MAVLinkPositionTargetLocalNed_t position_target;
} PositionTargetLocalNedSample_t;

typedef struct {
    HeartbeatSample_t *heartbeats;
    size_t heartbeat_count;
    size_t heartbeat_capacity;

    AttitudeSample_t *attitudes;
    size_t attitude_count;
    size_t attitude_capacity;

    AttitudeQuaternionSample_t *attitude_quaternions;
    size_t attitude_quaternion_count;
    size_t attitude_quaternion_capacity;

    LocalPositionNedSample_t *local_positions;
    size_t local_position_count;
    size_t local_position_capacity;

    GlobalPositionIntSample_t *global_positions;
    size_t global_position_count;
    size_t global_position_capacity;

    SetPositionTargetLocalNedSample_t *set_position_targets;
    size_t set_position_target_count;
    size_t set_position_target_capacity;

    PositionTargetLocalNedSample_t *position_targets;
    size_t position_target_count;
    size_t position_target_capacity;
} FlightData_t;

#endif
