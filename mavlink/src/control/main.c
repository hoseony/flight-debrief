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
#include "../../include/flight_data.h"
#include "../..//include/tlog.h"
#include "../../include/replay.h"

/* ---------- function prototype ---------- */
bool send_fixed_position_setpoint( int fd, const struct sockaddr_in *px4_address, socklen_t px4_address_length, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component);
static uint64_t monotonic_time_ms(void);

/* ---------- FSM (states) ---------- */
// FSM approach makes it a bit cleaner (believe or not...)
typedef enum {
    CONTROL_WAIT_HEARTBEAT,
    CONTROL_PRESTREAM_SETPOINT,
    CONTROL_WAIT_OFFBOARD_ACK,
    CONTROL_WAIT_ARM_ACK,
    CONTROL_HOLD_POSITION,
    CONTROL_REPLAY_TRAJECTORY,
    CONTROL_LAND,
    CONTROL_ERROR
} ControlState_t;

/* ---------- main function ---------- */
int main(int argc, char **argv) {
    // this part is basically integration of replay to the state machine that is in this file 
    // The code is indeed getting messy, but for now, bare with me...

    /* ---------- open the file ---------- */
    if (argc != 2) {
        fprintf(stderr, "usage: %s <telemetry.tlog>\n", argv[0]);
        return 1;
    }

    FILE *file = fopen(argv[1], "rb");

    if (file == NULL) {
        fprintf(stderr, "cannot open %s: %s\n", argv[1], strerror(errno));
        return 1;
    }

    /* ---------- load flight data ---------- */
    FlightData_t flight_data = {0};
    TLogLoadStats_t stats = {0};

    printf("READING %s\n", argv[1]);

    if (!tlog_load_flight_data(file, &flight_data, &stats)) {
        fprintf(stderr, "failed to load tlog\n");
        return 1;
    }

    /* ---------- convert it to trajectory ---------- */
    // check if initial timestamp exist
    if (flight_data.local_position_count == 0) {
        fprintf(stderr, "ERROR READING LOCAL_POSITION_COUNT: local_position_count = 0\n");
        flight_data_free(&flight_data);

        if (fclose(file) == EOF) {
            perror("fclose");
        }

        return 1;
    }

    ReplayTrajectory_t trajectory = {0};
    if (!replay_trajectory_from_flight_data(&flight_data, &trajectory)) {
        return 1;
    };

    ReplayTrajectory_t resample = {0};
    if (!replay_trajectory_resample(&trajectory, 100000, &resample)) {
        return 1;
    };

    // safety check!
    TrajectorySafetyLimit_t limits = {
        .maximum_altitude = 3.0f,
        .maximum_horizontal_distance = 0.5f,
        .speed = 1.0f,
        .total_duration_us = UINT64_C(25000000) 
    };
    
    if (!replay_trajectory_validate(&resample, &limits)) {
        return 1;
    }

    printf("Trajectory ready: %zu samples\n", resample.count);

    /* ---------- open port ---------- */
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


    /* ---------- FSM starts here ---------- */
    ControlState_t state = CONTROL_WAIT_HEARTBEAT;

    /* state: CONTROL_WAIT_HEARTBEAT */
    uint8_t buffer[2048];
    MAVLinkParser_t parser;
    MAVLinkFrame_t frame;
    mavlink_parser_init(&parser);

    struct sockaddr_in px4_address = {0};
    socklen_t px4_address_length = sizeof(px4_address);
    uint8_t target_system = 0;
    uint8_t target_component = 0;
    bool px4_found = false;

    /* state: CONTROL_PRESTREAM_SETPOINT */
    size_t setpoint_count = 0;
    
    // ID from this program
    const uint8_t source_system = 255;
    const uint8_t source_component = 190;

    /* state: CONTROL_REPLAY_TRAJECTORY */
    size_t replay_index = 0;

    /* command retry */
    const uint64_t command_retry_interval_ms = 500;
    const size_t command_max_attempts = 5;
    uint64_t offboard_last_send_ms = 0;
    uint64_t arm_last_send_ms = 0;
    size_t offboard_attempts = 0;
    size_t arm_attempts = 0;


    // sorry for a massive long lines of code, 
    // if I have some time, I will fix this to make it easier to read
    // but for now, it does work.... so..
    printf("Waiting for PX4 heartbeat...\n");

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
                    printf("Prestreaming 20 setpoints at 10 Hz...\n");
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
                if (setpoint_count >= 20) {
                    printf("Prestream complete: %zu setpoints sent\n", setpoint_count);

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
                    offboard_attempts = 1;
                    offboard_last_send_ms = monotonic_time_ms();
                    printf("Waiting for Offboard ACK...\n");
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
                        printf(
                            "COMMAND_ACK received: command=%u result=%u\n",
                            ack.command,
                            ack.result
                        );

                        if (ack.command != 176) {
                            continue;
                        }

                        if (ack.result == 0) {
                            printf("Offboard accepted\n");
                            // send arm command
                            MAVLinkEncodedFrame_t arm_command;

                            if (!mavlink_encode_arm(&arm_command,
                                        source_system, source_component,
                                        target_system, target_component)) {
                                fprintf(stderr, "failed to encode Arm command\n");
                                state = CONTROL_ERROR;
                                break;
                            }

                            ssize_t result = sendto(fd, arm_command.bytes, arm_command.length,
                                    0, (const struct sockaddr *)&px4_address, px4_address_length);
                            if (result != (ssize_t)arm_command.length) {
                                perror("arm command sent fail");
                                state = CONTROL_ERROR;
                                break;
                            }

                            arm_attempts = 1;
                            arm_last_send_ms = monotonic_time_ms();
                            printf("Arm command sent\n");
                            printf("Waiting for Arm ACK...\n");
                            state = CONTROL_WAIT_ARM_ACK;
                        } else {
                            fprintf(stderr, "Offboard rejected: result=%u\n", ack.result);
                            state = CONTROL_ERROR;
                        }
                    }
                }

                if (state == CONTROL_WAIT_OFFBOARD_ACK
                        && monotonic_time_ms() - offboard_last_send_ms
                            >= command_retry_interval_ms) {
                    if (offboard_attempts >= command_max_attempts) {
                        fprintf(stderr, "Offboard ACK timeout after %zu attempts\n",
                                offboard_attempts);
                        state = CONTROL_ERROR;
                        break;
                    }

                    MAVLinkEncodedFrame_t offboard_command;

                    if (!mavlink_encode_set_offboard_mode(&offboard_command,
                                source_system, source_component,
                                target_system, target_component)) {
                        fprintf(stderr, "failed to encode Offboard command retry\n");
                        state = CONTROL_ERROR;
                        break;
                    }

                    ssize_t sent = sendto(fd, offboard_command.bytes,
                            offboard_command.length, 0,
                            (const struct sockaddr *)&px4_address, px4_address_length);

                    if (sent != (ssize_t)offboard_command.length) {
                        perror("sendto Offboard command retry");
                        state = CONTROL_ERROR;
                        break;
                    }

                    offboard_attempts++;
                    offboard_last_send_ms = monotonic_time_ms();
                    printf("Offboard command retry %zu/%zu\n",
                            offboard_attempts, command_max_attempts);
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

                        printf(
                            "COMMAND_ACK received: command=%u result=%u\n",
                            ack.command,
                            ack.result
                        );

                        // this case is when its ack for different command
                        if (ack.command != 400) {
                            continue;
                        }

                        if (ack.result == 0) {
                            printf("Arm accepted\n");
                            // state = CONTROL_HOLD_POSITION;
                            printf("Replay starting: %zu samples at 10 Hz\n",
                                    resample.count);
                            state = CONTROL_REPLAY_TRAJECTORY;
                        } else {
                            fprintf(stderr, "arm rejected: result=%u\n", ack.result);
                            state = CONTROL_ERROR;
                        }
                    }
                }

                if (state == CONTROL_WAIT_ARM_ACK
                        && monotonic_time_ms() - arm_last_send_ms
                            >= command_retry_interval_ms) {
                    if (arm_attempts >= command_max_attempts) {
                        fprintf(stderr, "Arm ACK timeout after %zu attempts\n",
                                arm_attempts);
                        state = CONTROL_ERROR;
                        break;
                    }

                    MAVLinkEncodedFrame_t arm_command;

                    if (!mavlink_encode_arm(&arm_command,
                                source_system, source_component,
                                target_system, target_component)) {
                        fprintf(stderr, "failed to encode Arm command retry\n");
                        state = CONTROL_ERROR;
                        break;
                    }

                    ssize_t sent = sendto(fd, arm_command.bytes,
                            arm_command.length, 0,
                            (const struct sockaddr *)&px4_address, px4_address_length);

                    if (sent != (ssize_t)arm_command.length) {
                        perror("sendto Arm command retry");
                        state = CONTROL_ERROR;
                        break;
                    }

                    arm_attempts++;
                    arm_last_send_ms = monotonic_time_ms();
                    printf("Arm command retry %zu/%zu\n",
                            arm_attempts, command_max_attempts);
                }

                break;
            }

            case CONTROL_HOLD_POSITION: {
                MAVLinkEncodedFrame_t command;

                if (!mavlink_encode_position_target_local_ned(&command, source_system, source_component, 
                            target_system, target_component, 0, 0.0f, 0.0f,-1.0f)) {
                    state = CONTROL_ERROR;
                    break;
                }

                ssize_t sent = sendto( fd, command.bytes, command.length, 0,
                        (const struct sockaddr *)&px4_address, px4_address_length);

                if (!(sent == (ssize_t)command.length)) {
                    state = CONTROL_ERROR;
                    break;
                }

                break;
            }
            
            case CONTROL_REPLAY_TRAJECTORY: {
                
                if (replay_index >= resample.count) {
                    printf("Replay complete: %zu samples sent\n", resample.count);
                    state = CONTROL_HOLD_POSITION;
                    break;
                }

                ReplayPosition_t *position = &resample.positions[replay_index];
                MAVLinkEncodedFrame_t command;

                if (!mavlink_encode_position_target_local_ned(&command, source_system, source_component, 
                            target_system, target_component, 0, position->x, position->y,position->z)) {
                    state = CONTROL_ERROR;
                    break;
                }

                ssize_t sent = sendto( fd, command.bytes, command.length, 0,
                        (const struct sockaddr *)&px4_address, px4_address_length);

                if (!(sent == (ssize_t)command.length)) {
                    state = CONTROL_ERROR;
                    break;
                }

                if (replay_index % 10 == 0
                        || replay_index + 1 == resample.count) {
                    printf(
                        "Replay %zu/%zu t=%.1fs target=(%+.3f, %+.3f, %+.3f)\n",
                        replay_index + 1,
                        resample.count,
                        (double)position->elapsed_us / 1000000.0,
                        position->x,
                        position->y,
                        position->z
                    );
                }

                replay_index++;
                break;
            }

            case CONTROL_LAND: {
                break;
            }

            case CONTROL_ERROR:
                break;
        }
    }


    flight_data_free(&flight_data);
    replay_trajectory_free(&trajectory);
    replay_trajectory_free(&resample);

    return 0;
}

static uint64_t monotonic_time_ms(void) {
    struct timespec now = {0};
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (uint64_t)now.tv_sec * UINT64_C(1000)
        + (uint64_t)now.tv_nsec / UINT64_C(1000000);
}

bool send_fixed_position_setpoint( int fd, const struct sockaddr_in *px4_address, socklen_t px4_address_length, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component) {
    MAVLinkEncodedFrame_t command;

    if (!mavlink_encode_position_target_local_ned(&command, source_system, source_component, target_system, target_component, 0, 0.0f, 0.0f,-1.0f)) {
        return false;
    }

    ssize_t sent = sendto( fd, command.bytes, command.length, 0,
            (const struct sockaddr *)px4_address, px4_address_length);

    return (sent == (ssize_t)command.length);
}
