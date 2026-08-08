#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

/* Type Definitions */
typedef enum State {
    WAIT_MAGIC, 
    READ_FRAME,
} state_t;

// I directly took this from the documentation and modified a little to meet c syntax
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

// this is how it is ordered!
typedef struct {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} MAVLinkHeartbeat_t;

typedef struct {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} MAVLinkAttitude;

#endif
