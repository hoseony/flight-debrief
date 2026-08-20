#include <arpa/inet.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "../../include/mavlink_decode.h"
#include "../../include/mavlink_parser.h"
#include "../../include/udp_port.h"
#include "../../include/mavlink_encoder.h"

typedef enum {
    CONTROL_WAIT_HEARTBEAT,
    CONTROL_PRESTREAM_SETPOINT,
    CONTROL_WAIT_OFFBOARD_ACK,
    CONTROL_WAIT_ARM_ACK,
    CONTROL_HOLD_POSITION,
    CONTROL_ERROR
} ControlState_t;

static bool send_fixed_position_setpoint( int fd, const struct sockaddr_in *px4_address, socklen_t px4_address_length, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component) {
    MAVLinkEncodedFrame_t command;

    if (!mavlink_encode_position_target_local_ned(
            &command, source_system, source_component,
            target_system, target_component,
            0,
            0.0f, 0.0f,-1.0f)) {
        return false;
    }

    ssize_t sent = sendto(
        fd,
        command.bytes,
        command.length,
        0,
        (const struct sockaddr *)px4_address,
        px4_address_length
    );

    return (sent == (ssize_t)command.length);
}

int main(void) {
    /* open the port */
    int fd = udp_port_open(14550);

    if (fd < 0) {
        perror("serial_port_open");
        return 1;
    }

    // 10 Hz signal
    const struct timespec interval = {
        .tv_sec = 0,
        .tv_nsec = 100000000L
    };

    ControlState_t state = CONTROL_WAIT_HEARTBEAT;

    // for state CONTROL_WAIT_HEARTBEAT
    uint8_t buffer[2048];
    MAVLinkParser_t parser;
    MAVLinkFrame_t frame;
    mavlink_parser_init(&parser);

    struct sockaddr_in px4_address = {0};
    socklen_t px4_address_length = sizeof(px4_address);
    uint8_t target_system = 0;
    uint8_t target_component = 0;
    bool px4_found = false;

    // for state CONTROL_PRESTREAM_SETPOINT
    size_t setpoint_count = 0;
    
    // ID from this program
    const uint8_t source_system = 255;
    const uint8_t source_component = 190;


    while(true) {
        nanosleep(&interval, NULL);

        switch(state) {
            // waiting for the heartbeat from the drone 
            // to check the address it is coming from
            // once it finds the drone, it proceeds to next state
            case CONTROL_WAIT_HEARTBEAT: {
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

                /* parse until find px4 */
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

                    px4_address = sender;
                    px4_address_length = sender_length;
                    target_system = frame.bytes[5];
                    target_component = frame.bytes[6];
                    px4_found = true;

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

                // we found it!
                if (px4_found) {
                    state = CONTROL_PRESTREAM_SETPOINT;
                }

                break;
            }

            // Before sending offboard_command,
            // It needs to receive prestreaming for about 1s
            case CONTROL_PRESTREAM_SETPOINT: {
                if (!send_fixed_position_setpoint(
                        fd, &px4_address, px4_address_length,
                        source_system, source_component,
                        target_system, target_component)) {
                    state = CONTROL_ERROR;
                    break;
                }

                setpoint_count++;

                // after the prestreaming, send offboard_signal
                if (setpoint_count >= 10) {

                    MAVLinkEncodedFrame_t offboard_command;

                    if (!mavlink_encode_set_offboard_mode(&offboard_command,
                                source_system, source_component,
                                target_system, target_component)) {
                        fprintf(stderr, "failed to encode Offboard command\n");
                        state = CONTROL_ERROR;
                        break;
                    }

                    ssize_t offboard_sent = sendto(fd, offboard_command.bytes,
                            offboard_command.length, 0, 
                            (const struct sockaddr *)&px4_address,px4_address_length);

                    if (offboard_sent < 0) {
                        perror("sendto Offboard command");
                        state = CONTROL_ERROR;
                        break;
                    }

                    if ((size_t)offboard_sent != offboard_command.length) {
                        fprintf(stderr, "incomplete Offboard command send\n");
                        state = CONTROL_ERROR;
                        break;
                    }
                    
                    printf("Offboard command sent\n");
                    state = CONTROL_WAIT_OFFBOARD_ACK;
                }

                break;
            }

            // after sending offboard_command
            // you need to check if the physical drone has received
            // such message. This can be checked through ACK message types,
            // which, in this case COMMAND_ACK
            case CONTROL_WAIT_OFFBOARD_ACK: {
                if (!send_fixed_position_setpoint(
                        fd, &px4_address, px4_address_length,
                        source_system, source_component,
                        target_system, target_component)) {
                    state = CONTROL_ERROR;
                    break;
                }

                while (state == CONTROL_WAIT_OFFBOARD_ACK) {
                    struct sockaddr_in sender = {0};
                    socklen_t sender_length = sizeof(sender);

                    ssize_t received = recvfrom(fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                            (struct sockaddr *)&sender, &sender_length);

                    if (received < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        }

                        perror("recvfrom");
                        state = CONTROL_ERROR;
                        break;
                    }

                    // check if it is from px4
                    if (!(sender.sin_addr.s_addr == px4_address.sin_addr.s_addr
                                && sender.sin_port == px4_address.sin_port)) {
                        continue;
                    }

                    /* parse until find COMMAND_ACK */
                    for (ssize_t i = 0; i < received; i++) {
                        if (!mavlink_parser_consume(&parser, buffer[i], &frame)) {
                            continue;
                        }

                        if (frame_msgid(&frame) != 77) {
                            continue;
                        }

                        uint8_t crc_extra;
                        if (!mavlink_crc_extra_for(77, &crc_extra)
                                || !mavlink_frame_crc_valid(&frame, crc_extra)) {
                            continue;
                        }

                        MAVLinkCommandAck_t ack;

                        if (!mavlink_decode_command_ack(&frame, &ack)) {
                            continue;
                        }

                        // this case is when its ack for different command
                        if (ack.command != 176) {
                            continue;
                        }

                        if (ack.result == 0) {
                            printf("Offboard accepted\n");
                            // send arm command
                            MAVLinkEncodedFrame_t arm_command;

                            mavlink_encode_arm(&arm_command, source_system, source_component,
                                    target_system, target_component);

                            ssize_t result = sendto(fd, arm_command.bytes, arm_command.length,
                                    0, (const struct sockaddr *)&px4_address, px4_address_length);
                            if (result < 0) {
                                perror("arm command sent fail");
                                state = CONTROL_ERROR;
                                break;
                            }

                            state = CONTROL_WAIT_ARM_ACK;
                        } else {
                            fprintf(stderr, "Offboard rejected: result=%u\n", ack.result);
                            state = CONTROL_ERROR;
                        }
                    }
                }

                break;
            }

            case CONTROL_WAIT_ARM_ACK: {
                if (!send_fixed_position_setpoint(
                        fd, &px4_address, px4_address_length,
                        source_system, source_component,
                        target_system, target_component)) {
                    state = CONTROL_ERROR;
                    break;
                }

                while (state == CONTROL_WAIT_ARM_ACK) {
                    struct sockaddr_in sender = {0};
                    socklen_t sender_length = sizeof(sender);

                    ssize_t received = recvfrom(fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                            (struct sockaddr *)&sender, &sender_length);

                    if (received < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        }

                        perror("recvfrom");
                        state = CONTROL_ERROR;
                        break;
                    }

                    // check if it is from px4
                    if (!(sender.sin_addr.s_addr == px4_address.sin_addr.s_addr
                                && sender.sin_port == px4_address.sin_port)) {
                        continue;
                    }

                    /* parse until find COMMAND_ACK */
                    for (ssize_t i = 0; i < received; i++) {
                        if (!mavlink_parser_consume(&parser, buffer[i], &frame)) {
                            continue;
                        }

                        if (frame_msgid(&frame) != 77) {
                            continue;
                        }

                        uint8_t crc_extra;
                        if (!mavlink_crc_extra_for(77, &crc_extra)
                                || !mavlink_frame_crc_valid(&frame, crc_extra)) {
                            continue;
                        }

                        MAVLinkCommandAck_t ack;

                        if (!mavlink_decode_command_ack(&frame, &ack)) {
                            continue;
                        }

                        // this case is when its ack for different command
                        if (ack.command != 400) {
                            continue;
                        }

                        if (ack.result == 0) {
                            printf("Arm accepted\n");
                            state = CONTROL_HOLD_POSITION;
                        } else {
                            fprintf(stderr, "arm rejected: result=%u\n", ack.result);
                            state = CONTROL_ERROR;
                        }
                    }
                }

                break;
            }

            case CONTROL_HOLD_POSITION: {
               if (!send_fixed_position_setpoint(
                        fd, &px4_address, px4_address_length,
                        source_system, source_component,
                        target_system, target_component)) {
                    state = CONTROL_ERROR;
                    break;
                }

                break;
            }

            case CONTROL_ERROR:
                break;
        }
    }

    return 0;
}
