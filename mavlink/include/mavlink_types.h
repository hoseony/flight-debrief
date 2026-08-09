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


/// HEARTBEAT tells you that the system is still alive
typedef struct {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} MAVLinkHeartbeat_t;

/// Attitude of the system
typedef struct {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;

} MAVLinkAttitude_t;

#endif
