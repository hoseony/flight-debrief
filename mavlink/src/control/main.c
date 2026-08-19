#include <arpa/inet.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <libc.h>

#include "../../include/mavlink_decode.h"
#include "../../include/mavlink_parser.h"
#include "../../include/udp_port.h"
#include "../../include/mavlink_encoder.h"

int main(void) {
    int fd = udp_port_open(14550);

    if (fd < 0) {
        perror("serial_port_open");
        return 1;
    }

    MAVLinkParser_t parser;
    MAVLinkFrame_t frame;
    mavlink_parser_init(&parser);

    uint8_t buffer[2048];

    while(true) {
        struct sockaddr_in sender = {0};
        socklen_t sender_length = sizeof(sender);

        ssize_t received = recvfrom(fd, buffer, sizeof(buffer), 0,
                (struct sockaddr *)(&sender), &sender_length);

        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            
            perror("recvfrom");
            close(fd);
            return 1;
        }

        for (ssize_t i = 0; i < received; i++) {
            if (!mavlink_parser_consume(&parser, buffer[i], &frame)) {
                continue;
            }

            if (frame_msgid(&frame) != 0) {
                continue;
            }

            uint8_t crc_extra;
            if (!mavlink_crc_extra_for(0, &crc_extra) || !mavlink_frame_crc_valid(&frame, crc_extra)) {
                continue;
            }

            char address[INET_ADDRSTRLEN];

            inet_ntop(AF_INET, &sender.sin_addr, address, sizeof(address));

            printf(
                "PX4 heartbeat from %s:%u system=%u component=%u\n",
                address,
                ntohs(sender.sin_port),
                frame.bytes[5],
                frame.bytes[6]
            );
        }

        // ID from PX4
        uint8_t target_system = frame.bytes[5];
        uint8_t target_component = frame.bytes[6];
        
        // ID from this program
        const uint8_t source_system = 255;
        const uint8_t source_component = 190;
        
        // 10 Hz signal
        struct timespec interval = {
            .tv_sec = 0,
            .tv_nsec = 100000000L
        };

        MAVLinkEncodedFrame_t command;
        mavlink_encode_position_target_local_ned(
                &command,
                source_system,
                source_component,
                target_system,
                target_component,
                0,
                0.0f, 0.0f, -1.0f);

        sendto (fd, command.bytes, command.length, 0, (struct sockaddr *)&sender, sender_length);
        nanosleep(&interval, NULL);
    }
}
