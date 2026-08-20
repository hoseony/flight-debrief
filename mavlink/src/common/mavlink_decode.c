#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "../../include/mavlink_types.h"
#include "../../include/mavlink_decode.h"

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
/// For our usage, CRC must be started from 0xFFFF! (that's what the documentation says)
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

static bool copy_zero_padded_payload(const MAVLinkFrame_t *frame, uint32_t expected_msgid, uint8_t *payload, size_t payload_size) {
    if (payload == NULL || !frame_matches(frame, expected_msgid, 0)) {
        return false;
    }

    memset(payload, 0, payload_size);

    size_t copy_length = frame->bytes[1];

    if (copy_length > payload_size) {
        copy_length = payload_size;
    }

    memcpy(payload, &frame->bytes[10], copy_length);
    return true;
}

// from the mavlink implementation, I could make calculation function
// but for my case, it is faster and easier to straight bring the calculated constnats 
// for each message types.
bool mavlink_crc_extra_for(uint32_t message_id, uint8_t *crc_extra) {
    if (crc_extra == NULL) {
        return false;
    }

    switch (message_id) {
        case 0:
            *crc_extra = 50;
            return true;
        case 1:
            *crc_extra = 124;
            return true;
        case 4:
            *crc_extra = 237;
            return true;
        case 8:
            *crc_extra = 117;
            return true;
        case 24:
            *crc_extra = 24;
            return true;
        case 29:
            *crc_extra = 115;
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
        case 36:
            *crc_extra = 222;
            return true;
        case 42:
            *crc_extra = 28;
            return true;
        case 74:
            *crc_extra = 20;
            return true;
        case 76:
            *crc_extra = 152;
            return true;
        case 77:
            *crc_extra = 143;
            return true;
        case 83:
            *crc_extra = 22;
            return true;
        case 84:
            *crc_extra = 143;
            return true;
        case 85:
            *crc_extra = 140;
            return true;
        case 87:
            *crc_extra = 150;
            return true;
        case 141:
            *crc_extra = 47;
            return true;
        case 147:
            *crc_extra = 154;
            return true;
        case 230:
            *crc_extra = 163;
            return true;
        case 241:
            *crc_extra = 90;
            return true;
        case 242:
            *crc_extra = 104;
            return true;
        case 245:
            *crc_extra = 130;
            return true;
        case 290:
            *crc_extra = 251;
            return true;
        case 291:
            *crc_extra = 10;
            return true;
        case 380:
            *crc_extra = 232;
            return true;
        case 410:
            *crc_extra = 160;
            return true;
        case 411:
            *crc_extra = 106;
            return true;
        case 436:
            *crc_extra = 193;
            return true;
        case 441:
            *crc_extra = 169;
            return true;
        case 514:
            *crc_extra = 197;
            return true;
        case 12901:
            *crc_extra = 254;
            return true;
        case 12904:
            *crc_extra = 77;
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

bool mavlink_decode_command_ack(const MAVLinkFrame_t *frame, MAVLinkCommandAck_t *ack) {
    if (ack == NULL || !frame_matches(frame, 77, 3)) {
        return false;
    }

    ack->command = read_u16_le(&frame->bytes[10]);
    ack->result = frame->bytes[12];

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

bool mavlink_decode_servo_output_raw(const MAVLinkFrame_t *frame, MAVLinkServoOutputRaw_t *servo_output) {
    uint8_t payload[37];

    if (servo_output == NULL || !copy_zero_padded_payload(frame, 36, payload, sizeof(payload))) {
        return false;
    }

    servo_output->time_usec   = read_u32_le(&payload[0]);
    servo_output->servo1_raw  = read_u16_le(&payload[4]);
    servo_output->servo2_raw  = read_u16_le(&payload[6]);
    servo_output->servo3_raw  = read_u16_le(&payload[8]);
    servo_output->servo4_raw  = read_u16_le(&payload[10]);
    servo_output->servo5_raw  = read_u16_le(&payload[12]);
    servo_output->servo6_raw  = read_u16_le(&payload[14]);
    servo_output->servo7_raw  = read_u16_le(&payload[16]);
    servo_output->servo8_raw  = read_u16_le(&payload[18]);
    servo_output->port        = payload[20];
    servo_output->servo9_raw  = read_u16_le(&payload[21]);
    servo_output->servo10_raw = read_u16_le(&payload[23]);
    servo_output->servo11_raw = read_u16_le(&payload[25]);
    servo_output->servo12_raw = read_u16_le(&payload[27]);
    servo_output->servo13_raw = read_u16_le(&payload[29]);
    servo_output->servo14_raw = read_u16_le(&payload[31]);
    servo_output->servo15_raw = read_u16_le(&payload[33]);
    servo_output->servo16_raw = read_u16_le(&payload[35]);

    return true;
}

bool mavlink_decode_attitude_target(const MAVLinkFrame_t *frame, MAVLinkAttitudeTarget_t *attitude_target) {
    uint8_t payload[37];

    if (attitude_target == NULL || !copy_zero_padded_payload(frame, 83, payload, sizeof(payload))) {
        return false;
    }

    attitude_target->time_boot_ms    = read_u32_le(&payload[0]);
    attitude_target->q[0]            = read_f32_le(&payload[4]);
    attitude_target->q[1]            = read_f32_le(&payload[8]);
    attitude_target->q[2]            = read_f32_le(&payload[12]);
    attitude_target->q[3]            = read_f32_le(&payload[16]);
    attitude_target->body_roll_rate  = read_f32_le(&payload[20]);
    attitude_target->body_pitch_rate = read_f32_le(&payload[24]);
    attitude_target->body_yaw_rate   = read_f32_le(&payload[28]);
    attitude_target->thrust          = read_f32_le(&payload[32]);
    attitude_target->type_mask       = payload[36];

    return true;
}

bool mavlink_decode_setPositionTargetLocalNed(const MAVLinkFrame_t *frame, MAVLinkSetPositionTargetLocalNed_t *target) {
    if (target == NULL || !frame_matches(frame, 84, 53)) {
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
    target->target_system    = frame->bytes[60];
    target->target_component = frame->bytes[61];
    target->coordinate_frame = frame->bytes[62];

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
