#ifndef MAVLINK_TYPES_H
#define MAVLINK_TYPES_H

#include <stdint.h>
#include <stddef.h>

#define MAVLINK2_MAX_FRAME_LENGTH 280
#define MAVLINK2_MAGIC 0XFD
#define RAD_TO_DEG 57.2957795f

/* Type Definitions for MAVLink is in this file. */
// It includes definitions for both parser logic and types of actual message protocol

/* MAVLink Parsre */
typedef enum State {
    WAIT_MAGIC, 
    READ_FRAME,
} MAVLinkParserState_t;

typedef struct {
    MAVLinkParserState_t state;
    uint8_t bytes[MAVLINK2_MAX_FRAME_LENGTH];
    size_t position;
    size_t expected_length;
} MAVLinkParser_t;

typedef struct {
    uint8_t bytes[MAVLINK2_MAX_FRAME_LENGTH];
    size_t length;
} MAVLinkFrame_t;


/* MAVLink Packet & Message */
// These are directly took from the documentations (modified to meet the c syntax)
// Also, I tried to match the order of the variables as the actual order of the 
// bytes received to avoid confusion. 

typedef struct {
    uint8_t magic;              ///< protocol magic marker
    uint8_t len;                ///< Length of payload
    uint8_t incompat_flags;     ///< flags that must be understood
    uint8_t compat_flags;       ///< flags that can be ignored if not understood
    uint8_t seq;                ///< Sequence of packet
    uint8_t sysid;              ///< ID of message sender system/aircraft
    uint8_t compid;             ///< ID of the message sender component

    uint32_t msgid;
 /* uint8_t msgid 0:7;          ///< first 8 bits of the ID of the message
    uint8_t msgid 8:15;         ///< middle 8 bits of the ID of the message
    uint8_t msgid 16:23;        ///< last 8 bits of the ID of the message    */

    uint8_t payload[255];       ///< A maximum of 255 payload bytes
    uint16_t checksum;          ///< CRC-16/MCRF4XX

    uint8_t signature[13];
    int has_signature;
} MAVLinkPacket_t;


/* HEARTBEAT: message ID 0 */
typedef struct {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} MAVLinkHeartbeat_t;

/* COMMAND_ACK: message ID 77 */
typedef struct {
    uint16_t command;
    uint8_t result;
} MAVLinkCommandAck_t;

/* ATTITUDE: message ID 30 */
typedef struct {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} MAVLinkAttitude_t;

/* ATTITUDE_QUATERNION: message ID 31 */
typedef struct {
    uint32_t time_boot_ms;
    float q1;
    float q2;
    float q3;
    float q4;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} MAVLinkAttitudeQuaternion_t;

/* LOCAL_POSITION_NED: message ID 32 */
typedef struct {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
} MAVLinkLocalPositionNed_t;

/* GLOBAL_POSITION_INT: message ID 33 */
typedef struct {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg;
} MAVLinkGlobalPositionInt_t;

/* SERVO_OUTPUT_RAW: message ID 36 */
typedef struct {
    uint32_t time_usec;
    uint16_t servo1_raw;
    uint16_t servo2_raw;
    uint16_t servo3_raw;
    uint16_t servo4_raw;
    uint16_t servo5_raw;
    uint16_t servo6_raw;
    uint16_t servo7_raw;
    uint16_t servo8_raw;
    uint8_t port;
    uint16_t servo9_raw;
    uint16_t servo10_raw;
    uint16_t servo11_raw;
    uint16_t servo12_raw;
    uint16_t servo13_raw;
    uint16_t servo14_raw;
    uint16_t servo15_raw;
    uint16_t servo16_raw;
} MAVLinkServoOutputRaw_t;

/* ATTITUDE_TARGET: message ID 83 */
typedef struct {
    uint32_t time_boot_ms;
    float q[4];
    float body_roll_rate;
    float body_pitch_rate;
    float body_yaw_rate;
    float thrust;
    uint8_t type_mask;
} MAVLinkAttitudeTarget_t;

/* SET_POSITION_TARGET_LOCAL_NED: message ID 84 */
typedef struct {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float afx;
    float afy;
    float afz;
    float yaw;
    float yaw_rate;
    uint16_t type_mask;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t coordinate_frame;
} MAVLinkSetPositionTargetLocalNed_t;

/* POSITION_TARGET_LOCAL_NED: message ID 85 */
typedef struct {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float afx;
    float afy;
    float afz;
    float yaw;
    float yaw_rate;
    uint16_t type_mask;
    uint8_t coordinate_frame;
} MAVLinkPositionTargetLocalNed_t;

#endif
