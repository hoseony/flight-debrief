#ifndef MAVLINK_DECODE_H
#define MAVLINK_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#include "mavlink_types.h"

/* helper functions for mavlink decoder */
uint16_t read_u16_le(const uint8_t *bytes);
int16_t read_i16_le(const uint8_t *bytes);
uint32_t read_u32_le(const uint8_t *bytes);
int32_t read_i32_le(const uint8_t *bytes);
float read_f32_le(const uint8_t *bytes);

/* CRC validation related functions */
void crc_accumulate(uint8_t byte, uint16_t *crc);
uint16_t mavlink_frame_crc_calculate(const MAVLinkFrame_t *frame, uint8_t crc_extra);
bool mavlink_frame_crc_valid(const MAVLinkFrame_t *frame, uint8_t crc_extra);
uint32_t frame_msgid(const MAVLinkFrame_t *frame);
bool mavlink_crc_extra_for(uint32_t message_id, uint8_t *crc_extra);

/* MAVLink Decoder */
bool mavlink_decode_heartbeat(const MAVLinkFrame_t *frame, MAVLinkHeartbeat_t *heartbeat);
bool mavlink_decode_command_ack(const MAVLinkFrame_t *frame, MAVLinkCommandAck_t *ack);
bool mavlink_decode_attitude(const MAVLinkFrame_t *frame, MAVLinkAttitude_t *attitude);
bool mavlink_decode_attitudeQuaternion(const MAVLinkFrame_t *frame, MAVLinkAttitudeQuaternion_t *attitude_quaternion);
bool mavlink_decode_localPositionNed(const MAVLinkFrame_t *frame, MAVLinkLocalPositionNed_t *position);
bool mavlink_decode_globalPositionInt(const MAVLinkFrame_t *frame, MAVLinkGlobalPositionInt_t *position);
bool mavlink_decode_servo_output_raw(const MAVLinkFrame_t *frame, MAVLinkServoOutputRaw_t *servo_output);
bool mavlink_decode_attitude_target(const MAVLinkFrame_t *frame, MAVLinkAttitudeTarget_t *attitude_target);
bool mavlink_decode_setPositionTargetLocalNed(const MAVLinkFrame_t *frame, MAVLinkSetPositionTargetLocalNed_t *target);
bool mavlink_decode_positionTargetLocalNed(const MAVLinkFrame_t *frame, MAVLinkPositionTargetLocalNed_t *target);

#endif
