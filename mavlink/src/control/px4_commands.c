#include <sys/socket.h>

#include "../../include/px4_commands.h"

static bool send_encoded_frame(
        int fd,
        const struct sockaddr_in *px4_address,
        socklen_t px4_address_length,
        const MAVLinkEncodedFrame_t *frame) {
    if (px4_address == NULL || frame == NULL || frame->length == 0) {
        return false;
    }

    ssize_t sent = sendto(fd, frame->bytes, frame->length, 0,
            (const struct sockaddr *)px4_address, px4_address_length);

    return sent == (ssize_t)frame->length;
}

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
        float z) {
    MAVLinkEncodedFrame_t command;

    if (!mavlink_encode_position_target_local_ned(
                &command,
                source_system, source_component,
                target_system, target_component,
                0, x, y, z)) {
        return false;
    }

    return send_encoded_frame(fd, px4_address, px4_address_length, &command);
}

bool encode_and_send_command(
        int fd,
        const struct sockaddr_in *px4_address,
        socklen_t px4_address_length,
        uint8_t source_system,
        uint8_t source_component,
        uint8_t target_system,
        uint8_t target_component,
        CommandEncoderFn encoder) {
    if (encoder == NULL) {
        return false;
    }

    MAVLinkEncodedFrame_t frame;

    if (!encoder(
                &frame,
                source_system, source_component,
                target_system, target_component)) {
        return false;
    }

    return send_encoded_frame(fd, px4_address, px4_address_length, &frame);
}
