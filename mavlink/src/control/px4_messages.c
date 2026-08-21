#include <errno.h>
#include <stdio.h>
#include <sys/socket.h>

#include "../../include/mavlink_decode.h"
#include "../../include/px4_messages.h"

bool receive_px4_messages(
        int fd,
        const struct sockaddr_in *px4_address,
        MAVLinkParser_t *parser,
        PX4ReceivedMessages_t *received_messages) {
    if (px4_address == NULL || parser == NULL || received_messages == NULL) {
        return false;
    }

    uint8_t receive_buffer[2048];

    while (true) {
        struct sockaddr_in sender = {0};
        socklen_t sender_length = sizeof(sender);

        ssize_t received = recvfrom(fd,
                receive_buffer, sizeof(receive_buffer), MSG_DONTWAIT,
                (struct sockaddr *)&sender, &sender_length);

        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return true;
            }

            perror("receive_px4_messages: recvfrom");
            return false;
        }

        if (!(sender.sin_addr.s_addr == px4_address->sin_addr.s_addr
                    && sender.sin_port == px4_address->sin_port)) {
            continue;
        }

        MAVLinkFrame_t frame;

        for (ssize_t i = 0; i < received; i++) {
            if (!mavlink_parser_consume(parser, receive_buffer[i], &frame)) {
                continue;
            }

            // here is where all the decoding is happening
            uint32_t message_id = frame_msgid(&frame);
            if (message_id != 0 && message_id != 32 && message_id != 77) {
                continue;
            }

            uint8_t crc_extra;
            if (!mavlink_crc_extra_for(message_id, &crc_extra)
                    || !mavlink_frame_crc_valid(&frame, crc_extra)) {
                continue;
            }

            if (message_id == 0) {
                MAVLinkHeartbeat_t heartbeat;

                if (mavlink_decode_heartbeat(&frame, &heartbeat)) {
                    received_messages->heartbeat = heartbeat;
                    received_messages->heartbeat_received = true;
                }

                continue;
            }

            if (message_id == 32) {
                MAVLinkLocalPositionNed_t local_position;

                if (mavlink_decode_localPositionNed(&frame, &local_position)) {
                    received_messages->local_position = local_position;
                    received_messages->local_position_received = true;
                }

                continue;
            }

            MAVLinkCommandAck_t command_ack;

            if (mavlink_decode_command_ack(&frame, &command_ack)) {
                received_messages->command_ack = command_ack;
                received_messages->command_ack_received = true;
            }
        }
    }
}
