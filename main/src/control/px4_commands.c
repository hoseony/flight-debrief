#include "../../include/px4_commands.h"

static bool send_encoded_frame(
        ControlTransport_t *transport,
        const MAVLinkEncodedFrame_t *frame) {
    if (transport == NULL || frame == NULL || frame->length == 0) {
        return false;
    }

    return control_transport_send(transport, frame->bytes, frame->length);
}

bool send_position_setpoint(
        ControlTransport_t *transport,
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

    return send_encoded_frame(transport, &command);
}

bool encode_and_send_command(
        ControlTransport_t *transport,
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

    return send_encoded_frame(transport, &frame);
}
