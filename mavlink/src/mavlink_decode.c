#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "../include/mavlink_types.h"

// you can do this to ensure float to be 32 bits
_Static_assert( sizeof(float) == sizeof(uint32_t), "MAVLink requires a 32-bit float");


/* Helper Functions */

/// read 4 bytes consecutively and returns into one uint32_t
uint32_t read_u32_le(const uint8_t *bytes) {
    return ((uint32_t)bytes[0] | (uint32_t)(bytes[1] << 8) 
            | (uint32_t)(bytes[2] << 16) | (uint32_t)(bytes[3] << 24));
}

/// read 4 bytes consecutively and returns into one float
float read_f32_le(const uint8_t *bytes) {
    uint32_t bits = read_u32_le(bytes);
    float value;

    // memcpy!!
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* MAVLink Decoding Functions */

/// CRC algorithm
void crc_accumulate(unsigned char byte, uint16_t *crc) {
    unsigned char tmp;

    tmp = byte ^ (unsigned char)(*crc & 0xFF);
    tmp ^= (unsigned char)(tmp << 4);

    *crc = (*crc >> 8)
         ^ ((uint16_t)tmp << 8)
         ^ ((uint16_t)tmp << 3)
         ^ ((uint16_t)tmp >> 4);
}

uint32_t frame_msgid(const MAVLinkFrame_t *frame) {
    return (frame->bytes[7] | (frame->bytes[8] << 8) | (frame->bytes[9] << 16));
}

/* for each msg type that I am going to use */
bool mavlink_decode_heartbeat(const MAVLinkFrame_t *frame, MAVLinkHeartbeat_t heartbeat) {
    if (frame_msgid(frame) == 0 && frame->bytes[1] == 9) {
        
        heartbeat.custom_mode = 
            frame->bytes[10] | (frame->bytes[11] << 8) | (frame->bytes[12] << 16) | (frame->bytes[13] << 24);

        heartbeat.type            = frame->bytes[14];
        heartbeat.autopilot       = frame->bytes[15];
        heartbeat.base_mode       = frame->bytes[16];
        heartbeat.system_status   = frame->bytes[17];
        heartbeat.mavlink_version = frame->bytes[18];

    } else {
        return false;
    }

    return true;
}

bool mavlink_decode_attitude(const MAVLinkFrame_t *frame, MAVLinkAttitude_t *attitude) {
    if (frame_msgid(frame) == 30 && frame->bytes[1] == 28) {

        attitude->time_boot_ms = read_u32_le(&frame->bytes[10]);
        attitude->roll         = read_f32_le(&frame->bytes[14]);
        attitude->pitch        = read_f32_le(&frame->bytes[18]);
        attitude->yaw          = read_f32_le(&frame->bytes[22]);
        attitude->rollspeed    = read_f32_le(&frame->bytes[26]);
        attitude->pitchspeed   = read_f32_le(&frame->bytes[30]);
        attitude->yawspeed     = read_f32_le(&frame->bytes[34]);
    } else {
        return false;
    }

    return true;
}
