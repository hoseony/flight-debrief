#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "../include/mavlink_types.h"
#include "../include/mavlink_decode.h"

// you can do this to ensure float to be 32 bits
_Static_assert( sizeof(float) == sizeof(uint32_t), "MAVLink requires a 32-bit float");


/* Helper Functions */
uint16_t read_u16_le(const uint8_t *bytes) {
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

int16_t read_i16_le(const uint8_t *bytes) {
    uint16_t bits = read_u16_le(bytes);
    int16_t value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

/// read 4 bytes consecutively and returns into one uint32_t
uint32_t read_u32_le(const uint8_t *bytes) {
    return ((uint32_t)bytes[0] 
            | ((uint32_t)bytes[1] << 8)
            | ((uint32_t)bytes[2] << 16) 
            | ((uint32_t)bytes[3] << 24));
}

/// read 4 bytes consecutively and returns into one float
float read_f32_le(const uint8_t *bytes) {
    uint32_t bits = read_u32_le(bytes);
    float value;

    // memcpy!!
    memcpy(&value, &bits, sizeof(value));
    return value;
}

int32_t read_i32_le(const uint8_t *bytes) {
    uint32_t bits = read_u32_le(bytes);
    int32_t value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}


/* MAVLink Decoding Functions */

/* CRC Validation */
/// CRC algorithm
/// https://github.com/mavlink/c_library_v2/blob/master/checksum.h
void crc_accumulate(unsigned char byte, uint16_t *crc) {
    unsigned char tmp;

    tmp = byte ^ (uint8_t)(*crc & 0xFF);
    tmp ^= tmp << 4;

    *crc = (*crc >> 8)
         ^ ((uint16_t)tmp << 8)
         ^ ((uint16_t)tmp << 3)
         ^ ((uint16_t)tmp >> 4);
}

uint16_t mavlink_frame_crc_calculate(const MAVLinkFrame_t *frame, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;
    size_t crc_position = 10 + frame->bytes[1];

    for (size_t i = 1; i < crc_position; i++) {
        crc_accumulate(frame->bytes[i], &crc);
    }

    crc_accumulate(crc_extra, &crc);

    return crc;
}

bool mavlink_frame_crc_valid(const MAVLinkFrame_t *frame, uint8_t crc_extra) {
    if (frame == NULL || frame->length < 12u) {
        return false;
    }

    size_t crc_position = 10 + frame->bytes[1];
   
    if (crc_position + 2 > frame->length) {
        return false;
    }

    uint16_t received_crc 
        = (uint16_t)frame->bytes[crc_position] 
        | ((uint16_t)frame->bytes[crc_position + 1] << 8);

    uint16_t calculated_crc = mavlink_frame_crc_calculate(frame, crc_extra);

    return calculated_crc == received_crc;
}


/// error handling function for mavlink decoder
static bool frame_matches(const MAVLinkFrame_t *frame, uint32_t expected_msgid, uint8_t minimum_payload_length) {
    if (frame == NULL) {
        return false;
    }

    if (frame->length < 12u) {
        return false;
    }

    if (frame->bytes[0] != MAVLINK2_MAGIC) {
        return false;
    }

    if (frame_msgid(frame) != expected_msgid) {
        return false;
    }

    if (frame->bytes[1] < minimum_payload_length) {
        return false;
    }

    if (frame->length < 12u + frame->bytes[1]) {
        return false;
    }

    return true;
}

bool mavlink_crc_extra_for(uint32_t message_id, uint8_t *crc_extra) {
    if (crc_extra == NULL) {
        return false;
    }

    switch (message_id) {
        case 0:
            *crc_extra = 50;
            return true;
        case 30:
            *crc_extra = 39;
            return true;
        case 31:
            *crc_extra = 246;
            return true;
        case 32:
            *crc_extra = 185;
            return true;
        case 33:
            *crc_extra = 104;
            return true;
        case 85:
            *crc_extra = 140;
            return true;
        default:
            return false;
    }
}


/* MAVLINK_DECODE */
// for each msg type that I am going to use

uint32_t frame_msgid(const MAVLinkFrame_t *frame) {
    return ((uint32_t)frame->bytes[7] | ((uint32_t)frame->bytes[8] << 8) | ((uint32_t)frame->bytes[9] << 16));
}

bool mavlink_decode_heartbeat(const MAVLinkFrame_t *frame, MAVLinkHeartbeat_t *heartbeat) {
    if (heartbeat == NULL || !frame_matches(frame, 0, 9)) {
        return false;
    }

    heartbeat->custom_mode     = read_u32_le(&frame->bytes[10]);
    heartbeat->type            = frame->bytes[14];
    heartbeat->autopilot       = frame->bytes[15];
    heartbeat->base_mode       = frame->bytes[16];
    heartbeat->system_status   = frame->bytes[17];
    heartbeat->mavlink_version = frame->bytes[18];

    return true;
}

bool mavlink_decode_attitude(const MAVLinkFrame_t *frame, MAVLinkAttitude_t *attitude) {
    if (attitude == NULL || !frame_matches(frame, 30, 28)) {
        return false;
    }

    attitude->time_boot_ms = read_u32_le(&frame->bytes[10]);
    attitude->roll         = read_f32_le(&frame->bytes[14]);
    attitude->pitch        = read_f32_le(&frame->bytes[18]);
    attitude->yaw          = read_f32_le(&frame->bytes[22]);
    attitude->rollspeed    = read_f32_le(&frame->bytes[26]);
    attitude->pitchspeed   = read_f32_le(&frame->bytes[30]);
    attitude->yawspeed     = read_f32_le(&frame->bytes[34]);

    return true;
}

bool mavlink_decode_attitudeQuaternion(const MAVLinkFrame_t *frame, MAVLinkAttitudeQuaternion_t *attitude_quaternion) {
    if (attitude_quaternion== NULL || !frame_matches(frame, 31, 32)) {
        return false;
    }

    attitude_quaternion->time_boot_ms = read_u32_le(&frame->bytes[10]);
    attitude_quaternion->q1           = read_f32_le(&frame->bytes[14]);
    attitude_quaternion->q2           = read_f32_le(&frame->bytes[18]);
    attitude_quaternion->q3           = read_f32_le(&frame->bytes[22]);
    attitude_quaternion->q4           = read_f32_le(&frame->bytes[26]);
    attitude_quaternion->rollspeed    = read_f32_le(&frame->bytes[30]);
    attitude_quaternion->pitchspeed   = read_f32_le(&frame->bytes[34]);
    attitude_quaternion->yawspeed     = read_f32_le(&frame->bytes[38]);
    /// for now I'm ignoring the extension...

    return true;
}

bool mavlink_decode_localPositionNed(const MAVLinkFrame_t *frame, MAVLinkLocalPositionNed_t *local_position_ned) {
    if (local_position_ned == NULL || !frame_matches(frame, 32, 28)) {
        return false;
    }

    local_position_ned->time_boot_ms = read_u32_le(&frame->bytes[10]);
    local_position_ned->x            = read_f32_le(&frame->bytes[14]);
    local_position_ned->y            = read_f32_le(&frame->bytes[18]);
    local_position_ned->z            = read_f32_le(&frame->bytes[22]);
    local_position_ned->vx           = read_f32_le(&frame->bytes[26]);
    local_position_ned->vy           = read_f32_le(&frame->bytes[30]);
    local_position_ned->vz           = read_f32_le(&frame->bytes[34]);

    return true;
} 


bool mavlink_decode_globalPositionInt(const MAVLinkFrame_t *frame, MAVLinkGlobalPositionInt_t *position){
    if (position == NULL || !frame_matches(frame, 33, 28)) {
        return false;
    }

    position->time_boot_ms = read_u32_le(&frame->bytes[10]);
    position->lat          = read_i32_le(&frame->bytes[14]);
    position->lon          = read_i32_le(&frame->bytes[18]);
    position->alt          = read_i32_le(&frame->bytes[22]);
    position->relative_alt = read_i32_le(&frame->bytes[26]);
    position->vx           = read_i16_le(&frame->bytes[30]);
    position->vy           = read_i16_le(&frame->bytes[32]);
    position->vz           = read_i16_le(&frame->bytes[34]);
    position->hdg          = read_u16_le(&frame->bytes[36]);

    return true;
}

bool mavlink_decode_positionTargetLocalNed(const MAVLinkFrame_t *frame, MAVLinkPositionTargetLocalNed_t *target) {
    if (target == NULL || !frame_matches(frame, 85, 51)) {
        return false;
    }

    target->time_boot_ms     = read_u32_le(&frame->bytes[10]);
    target->x                = read_f32_le(&frame->bytes[14]);
    target->y                = read_f32_le(&frame->bytes[18]);
    target->z                = read_f32_le(&frame->bytes[22]);
    target->vx               = read_f32_le(&frame->bytes[26]);
    target->vy               = read_f32_le(&frame->bytes[30]);
    target->vz               = read_f32_le(&frame->bytes[34]);
    target->afx              = read_f32_le(&frame->bytes[38]);
    target->afy              = read_f32_le(&frame->bytes[42]);
    target->afz              = read_f32_le(&frame->bytes[46]);
    target->yaw              = read_f32_le(&frame->bytes[50]);
    target->yaw_rate         = read_f32_le(&frame->bytes[54]);
    target->type_mask        = read_u16_le(&frame->bytes[58]);
    target->coordinate_frame = frame->bytes[60];

    return true;
}
