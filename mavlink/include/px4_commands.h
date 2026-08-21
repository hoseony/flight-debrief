#ifndef PX4_COMMANDS_H
#define PX4_COMMANDS_H

#include <arpa/inet.h>
#include <stdbool.h>
#include <stdint.h>

#include "mavlink_encoder.h"

typedef bool (*CommandEncoderFn)(
    MAVLinkEncodedFrame_t *output,
    uint8_t source_system,
    uint8_t source_component,
    uint8_t target_system,
    uint8_t target_component
);

bool send_position_setpoint(
    int fd,
    const struct sockaddr_in *px4_address,
    socklen_t px4_address_length,
    uint8_t source_system,
    uint8_t source_component,
    uint8_t target_system,
    uint8_t target_component,
    float x,
    float y,
    float z
);

bool encode_and_send_command(
    int fd,
    const struct sockaddr_in *px4_address,
    socklen_t px4_address_length,
    uint8_t source_system,
    uint8_t source_component,
    uint8_t target_system,
    uint8_t target_component,
    CommandEncoderFn encoder
);

#endif
