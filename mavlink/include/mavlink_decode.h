#ifndef MAVLINK_DECODE_H
#define MAVLINK_DECODE_H

#include <stdint.h>
#include "mavlink_types.h"

uint32_t read_u32_le(const uint8_t *bytes);
float read_f32_le(const uint8_t *bytes); void crc_accumulate(unsigned char byte, uint16_t *crc);

void crc_accumulate(unsigned char byte, uint16_t *crc);
uint16_t mavlink_frame_crc_calculate(const MAVLinkFrame_t *frame, uint8_t crc_extra);
bool mavlink_frame_crc_valid(const MAVLinkFrame_t *frame, uint8_t crc_extra);

uint32_t frame_msgid(const MAVLinkFrame_t *frame);
bool mavlink_decode_heartbeat(const MAVLinkFrame_t *frame, MAVLinkHeartbeat_t heartbeat);
bool mavlink_decode_attitude(const MAVLinkFrame_t *frame, MAVLinkAttitude_t *attitude);

#endif
