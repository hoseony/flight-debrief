#ifndef MAVLINK_ENCODER_H
#define MAVLINK_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAVLINK_ENCODED_FRAME_CAPACITY 280

typedef struct {
    uint8_t bytes[MAVLINK_ENCODED_FRAME_CAPACITY];
    size_t length;
} MAVLinkEncodedFrame_t;

bool mavlink_encode_position_target_local_ned(
    MAVLinkEncodedFrame_t *output,
    uint8_t source_system,
    uint8_t source_component,
    uint8_t target_system,
    uint8_t target_component,
    uint32_t time_boot_ms,
    float x,
    float y,
    float z
);

bool mavlink_encode_set_offboard_mode(
    MAVLinkEncodedFrame_t *output,
    uint8_t source_system,
    uint8_t source_component,
    uint8_t target_system,
    uint8_t target_component
);

bool mavlink_encode_arm(
    MAVLinkEncodedFrame_t *output,
    uint8_t source_system,
    uint8_t source_component,
    uint8_t target_system,
    uint8_t target_component
);

#endif
