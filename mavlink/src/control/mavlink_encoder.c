#include <development/mavlink.h>
#include <math.h>
#include "../../include/mavlink_encoder.h"

_Static_assert(
    MAVLINK_ENCODED_FRAME_CAPACITY >= MAVLINK_MAX_PACKET_LEN,
    "encoded frame buffer is too small"
);

// I will be using the official mavlink library here since the deadline is getting tight!
// also for the safety reason...
//
// This is just a little wrapper thing that I had to do becuase of how the project is made 
// (some duplicate symbols and stuff...)

bool mavlink_encode_position_target_local_ned(MAVLinkEncodedFrame_t *output, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, float x, float y, float z) {
    if (output == NULL ||
        !isfinite(x) ||
        !isfinite(y) ||
        !isfinite(z)) {
        return false;
    }

    output->length = 0;

    mavlink_message_t message = {0};

    uint16_t type_mask =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    mavlink_msg_set_position_target_local_ned_pack(
        source_system, source_component,
        &message, time_boot_ms,
        target_system, target_component,
        MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f
    );

    output->length = mavlink_msg_to_send_buffer(output->bytes, &message);

    return output->length > 0;
}

bool mavlink_encode_set_offboard_mode( MAVLinkEncodedFrame_t *output, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component) {
    if (output == NULL) {
        return false;
    }

    output->length = 0;

    mavlink_message_t message = {0};

    mavlink_msg_command_long_pack(
        source_system, source_component,
        &message,
        target_system, target_component,
        MAV_CMD_DO_SET_MODE,
        0, (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );

    output->length = mavlink_msg_to_send_buffer(output->bytes, &message);

    return output->length > 0;
}

bool mavlink_encode_arm(MAVLinkEncodedFrame_t *output, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component) {
    if (output == NULL) {
        return false;
    }

    output->length = 0;

    mavlink_message_t message = {0};

    mavlink_msg_command_long_pack(
        source_system, source_component,
        &message,
        target_system, target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );

    output->length = mavlink_msg_to_send_buffer(output->bytes, &message);

    return output->length > 0;
}

bool mavlink_encode_land(MAVLinkEncodedFrame_t *output, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component) {
    if (output == NULL) {
        return false;
    }

    output->length = 0;

    mavlink_message_t message = {0};

    mavlink_msg_command_long_pack(
        source_system, source_component,
        &message,
        target_system, target_component,
        MAV_CMD_NAV_LAND,
        0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );

    output->length = mavlink_msg_to_send_buffer(output->bytes, &message);

    return output->length > 0;
}
