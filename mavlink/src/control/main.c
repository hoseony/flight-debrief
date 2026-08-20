/* Some notes for this code right here...
 * 
 * This Program is a replay for SITL (software in a loop).
 * It uses Gazebo x500 model and udp port to simulate the control 
 * logic.
 *
 * 1. load and prepare the replay (with safety limit check)
 * 2. open udp socket 
 * 3. start the event loop
 *  - poll() waits until 
 *      a. udp packet arrives 
 *         -> wake, receive datagrams 
 *         -> parse Heartbeat or command_ack
 *         -> change the control state if necessary
 *      b. next 10hz deadline arrives
 *         -> send the setpoint 
 *         -> move the deadline forward 100ms
 * 4. rest is state machine
 *
 * Overall Flow: 
 *      Load trajectory          -> Wait for Heartbeat
 *      -> Prestream setpoints   -> Request Offboard
 *      -> Wait for Offboard ACK -> Request Arm
 *      -> Wait for Arm ACK      -> Replay trajectory at 10 Hz
 *      -> Hold final position   -> Land
 */

#include <arpa/inet.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <poll.h>

#include "../../include/mavlink_decode.h"
#include "../../include/mavlink_parser.h"
#include "../../include/udp_port.h"
#include "../../include/mavlink_encoder.h"
#include "../../include/flight_data.h"
#include "../..//include/tlog.h"
#include "../../include/replay.h"

#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_CYAN "\033[36m"
#define COLOR_RESET "\033[0m"

#define LOG_ERROR COLOR_RED "[ERROR]" COLOR_RESET
#define LOG_ACK COLOR_GREEN "[ACK]" COLOR_RESET
#define LOG_STATE COLOR_CYAN "[STATE]" COLOR_RESET

/* ---------- function prototype ---------- */
static uint64_t monotonic_time_ms(void);
static bool send_position_setpoint(
        int fd, const struct sockaddr_in *px4_address,
        socklen_t px4_address_length,
        uint8_t source_system, uint8_t source_component,
        uint8_t target_system, uint8_t target_component,
        float x, float y, float z);
static bool drain_udp_datagrams(int fd);

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
        fprintf(stderr, "Usage: %s <telemetry.tlog>\n", argv[0]);
        return 1;
    }

    FILE *file = fopen(argv[1], "rb");

    if (file == NULL) {
        fprintf(stderr, LOG_ERROR " Failed to open log '%s': %s\n", argv[1], strerror(errno));
        return 1;
    }

    /* ---------- load flight data ---------- */
    FlightData_t flight_data = {0};
    TLogLoadStats_t stats = {0};

    printf("[INFO] Loading trajectory: %s\n", argv[1]);

    if (!tlog_load_flight_data(file, &flight_data, &stats)) {
        fprintf(stderr, LOG_ERROR " Failed to load tlog\n");
        return 1;
    }

    /* ---------- convert it to trajectory ---------- */
    // check if initial timestamp exist
    if (flight_data.local_position_count == 0) {
        fprintf(stderr, LOG_ERROR " Trajectory contains no local-position samples\n");
        flight_data_free(&flight_data);

        if (fclose(file) == EOF) {
            perror(LOG_ERROR " Failed to close telemetry log");
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
        .total_duration_us = UINT64_C(30000000) 
    };
    
    if (!replay_trajectory_validate(&resample, &limits)) {
        return 1;
    }

    printf("[INFO] Trajectory ready: samples=%zu\n", resample.count);

    /* ---------- open port ---------- */
    int fd = udp_port_open(14550);

    if (fd < 0) {
        perror(LOG_ERROR " Failed to open UDP port");
        return 1;
    }

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


    /* command last pos */
    ReplayPosition_t last_commanded_position;
    bool has_last_commanded_position = false;
    
    // sorry for a massive long lines of code, 
    // if I have some time, I will fix this to make it easier to read
    // but for now, it does work.... so..
    printf(LOG_STATE " Waiting for PX4 heartbeat\n");

    uint64_t next_setpoint_ms = monotonic_time_ms() + 100;


    /* ---------- MAIN LOOP ---------- */
    while (state != CONTROL_ERROR) {
        uint64_t now_ms = monotonic_time_ms();
        int timeout_ms = -1;

        // for the states that are not CONTROL_WAIT_HEARTBEAT
        // set a timeout
        if (state != CONTROL_WAIT_HEARTBEAT) {
            timeout_ms = (next_setpoint_ms > now_ms) ? (int)(next_setpoint_ms - now_ms) : 0;
        }

        struct pollfd events[2]= {
            {
                .fd = fd,
                // at least one udp datagram wiating
                .events = POLLIN
            },
            {
                .fd = STDIN_FILENO,
                .events = POLLIN
            }
        };
        
        // poll is conditional timeout 
        // thiscase when the "timeout" is passed or "POLLIN"
        int poll_result = poll(events, 2, timeout_ms);

        // error handling for poll
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }

            perror(LOG_ERROR " poll failed");
            state = CONTROL_ERROR;
            break;
        }

        // catch socket event error
        if (events[0].revents & (POLLERR | POLLNVAL)) {
            fprintf(stderr, LOG_ERROR " UDP socket event failure: revents=0x%x\n",
                    events[0].revents);
            state = CONTROL_ERROR;
            break;
        }

        if (events[1].revents & POLLNVAL) {
            fprintf(stderr, LOG_ERROR " Standard input is unavailable\n");
            state = CONTROL_ERROR;
            break;
        }

        // This is when you can read the socket. 
        // It ensures there exists some data
        bool socket_ready = (poll_result > 0) && (events[0].revents & POLLIN);
        bool input_ready = (poll_result > 0) && (events[1].revents & POLLIN);
        // update now
        now_ms = monotonic_time_ms();

        // condition that checks if it needs to send setpoint signal
        bool setpoint_due = (state != CONTROL_WAIT_HEARTBEAT) && (now_ms >= next_setpoint_ms);

        // for the states that actually uses incoming packets,
        // that is those that are waiting the signals from the drone
        if (socket_ready 
                && (state != CONTROL_WAIT_HEARTBEAT) && (state != CONTROL_WAIT_OFFBOARD_ACK) && (state != CONTROL_WAIT_ARM_ACK)) {
            // drain the udp
            if (!drain_udp_datagrams(fd)) {
                state = CONTROL_ERROR;
                break;
            }
        }

        switch(state) {
            // waiting for the heartbeat from the drone 
            // to check the address it is coming from
            // once it finds the drone, it proceeds to next state
            case CONTROL_WAIT_HEARTBEAT: {
                if (!socket_ready) {
                    break;
                }

                struct sockaddr_in sender = {0};
                socklen_t sender_length = sizeof(sender);

                ssize_t received = recvfrom(fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                        (struct sockaddr *)(&sender), &sender_length);

                if (received < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        break;
                    }
                    
                    perror(LOG_ERROR " Failed to receive UDP datagram");
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
                        "[INFO] PX4 discovered: address=%s:%u system=%u component=%u\n",
                        address,
                        ntohs(sender.sin_port),
                        frame.bytes[5],
                        frame.bytes[6]
                    );
                }

                // we found it!
                if (px4_found) {
                    printf(LOG_STATE " Prestreaming setpoints: count=20 rate=10Hz\n");
                    next_setpoint_ms = monotonic_time_ms();
                    state = CONTROL_PRESTREAM_SETPOINT;
                }

                break;
            }

            // Before sending offboard_command,
            // It needs to receive prestreaming for about 1s
            case CONTROL_PRESTREAM_SETPOINT: {
                if (!setpoint_due) {
                    break;
                }

                if (!send_position_setpoint(
                            fd, &px4_address, px4_address_length,
                            source_system, source_component,
                            target_system, target_component,
                            0.0f, 0.0f, -1.0f)) {
                    state = CONTROL_ERROR;
                    break;
                }

                setpoint_count++;

                // after the prestreaming, send offboard_signal
                if (setpoint_count >= 20) {
                    printf("[INFO] Prestream complete: sent=%zu\n", setpoint_count);

                    MAVLinkEncodedFrame_t offboard_command;

                    if (!mavlink_encode_set_offboard_mode(&offboard_command,
                                source_system, source_component,
                                target_system, target_component)) {
                        fprintf(stderr, LOG_ERROR " Failed to encode Offboard command\n");
                        state = CONTROL_ERROR;
                        break;
                    }

                    ssize_t offboard_sent = sendto(fd, offboard_command.bytes,
                            offboard_command.length, 0, 
                            (const struct sockaddr *)&px4_address,px4_address_length);

                    if (offboard_sent < 0) {
                        perror(LOG_ERROR " Failed to send Offboard command");
                        state = CONTROL_ERROR;
                        break;
                    }

                    if ((size_t)offboard_sent != offboard_command.length) {
                        fprintf(stderr, LOG_ERROR " Incomplete Offboard command transmission\n");
                        state = CONTROL_ERROR;
                        break;
                    }
                    
                    printf(LOG_STATE " Offboard mode requested: attempt=1/%zu\n",
                            command_max_attempts);
                    offboard_attempts = 1;
                    offboard_last_send_ms = monotonic_time_ms();
                    printf(LOG_STATE " Waiting for Offboard acknowledgement\n");
                    state = CONTROL_WAIT_OFFBOARD_ACK;
                }

                break;
            }

            // after sending offboard_command
            // you need to check if the physical drone has received
            // such message. This can be checked through ACK message types,
            // which, in this case COMMAND_ACK
            case CONTROL_WAIT_OFFBOARD_ACK: {
                if (setpoint_due && !send_position_setpoint(
                            fd, &px4_address, px4_address_length,
                            source_system, source_component,
                            target_system, target_component,
                            0.0f, 0.0f, -1.0f)) {
                    state = CONTROL_ERROR;
                    break;
                }

                while (socket_ready && state == CONTROL_WAIT_OFFBOARD_ACK) {
                    struct sockaddr_in sender = {0};
                    socklen_t sender_length = sizeof(sender);

                    ssize_t received = recvfrom(fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                            (struct sockaddr *)&sender, &sender_length);

                    if (received < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        }

                        perror(LOG_ERROR " Failed to receive UDP datagram");
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
                            LOG_ACK " command=%u result=%u\n",
                            ack.command,
                            ack.result
                        );

                        if (ack.command != 176) {
                            continue;
                        }

                        if (ack.result == 0) {
                            printf(LOG_STATE " Offboard mode accepted\n");
                            
                            // send arm command
                            MAVLinkEncodedFrame_t arm_command;

                            if (!mavlink_encode_arm(&arm_command,
                                        source_system, source_component,
                                        target_system, target_component)) {
                                fprintf(stderr, LOG_ERROR " Failed to encode Arm command\n");
                                state = CONTROL_ERROR;
                                break;
                            }

                            ssize_t result = sendto(fd, arm_command.bytes, arm_command.length,
                                    0, (const struct sockaddr *)&px4_address, px4_address_length);
                            if (result != (ssize_t)arm_command.length) {
                                perror(LOG_ERROR " Failed to send Arm command");
                                state = CONTROL_ERROR;
                                break;
                            }

                            arm_attempts = 1;
                            arm_last_send_ms = monotonic_time_ms();
                            printf(LOG_STATE " Vehicle arm requested: attempt=1/%zu\n",
                                    command_max_attempts);
                            printf(LOG_STATE " Waiting for Arm acknowledgement\n");
                            state = CONTROL_WAIT_ARM_ACK;
                        } else {
                            fprintf(stderr, LOG_ERROR " Offboard mode rejected: result=%u\n",
                                    ack.result);
                            state = CONTROL_ERROR;
                        }
                    }
                }

                if (state == CONTROL_WAIT_OFFBOARD_ACK
                        && monotonic_time_ms() - offboard_last_send_ms
                            >= command_retry_interval_ms) {
                    if (offboard_attempts >= command_max_attempts) {
                        fprintf(stderr, LOG_ERROR " Offboard acknowledgement timed out: attempts=%zu\n",
                                offboard_attempts);
                        state = CONTROL_ERROR;
                        break;
                    }

                    MAVLinkEncodedFrame_t offboard_command;

                    if (!mavlink_encode_set_offboard_mode(&offboard_command,
                                source_system, source_component,
                                target_system, target_component)) {
                        fprintf(stderr, LOG_ERROR " Failed to encode Offboard command retry\n");
                        state = CONTROL_ERROR;
                        break;
                    }

                    ssize_t sent = sendto(fd, offboard_command.bytes,
                            offboard_command.length, 0,
                            (const struct sockaddr *)&px4_address, px4_address_length);

                    if (sent != (ssize_t)offboard_command.length) {
                        perror(LOG_ERROR " Failed to resend Offboard command");
                        state = CONTROL_ERROR;
                        break;
                    }

                    offboard_attempts++;
                    offboard_last_send_ms = monotonic_time_ms();
                    printf(LOG_STATE " Offboard mode requested: attempt=%zu/%zu\n",
                            offboard_attempts, command_max_attempts);
                }

                break;
            }

            case CONTROL_WAIT_ARM_ACK: {
                if (setpoint_due && !send_position_setpoint(
                            fd, &px4_address, px4_address_length,
                            source_system, source_component,
                            target_system, target_component,
                            0.0f, 0.0f, -1.0f)) {
                    state = CONTROL_ERROR;
                    break;
                }

                while (socket_ready && state == CONTROL_WAIT_ARM_ACK) {
                    struct sockaddr_in sender = {0};
                    socklen_t sender_length = sizeof(sender);

                    ssize_t received = recvfrom(fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                            (struct sockaddr *)&sender, &sender_length);

                    if (received < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        }

                        perror(LOG_ERROR " Failed to receive UDP datagram");
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
                            LOG_ACK " command=%u result=%u\n",
                            ack.command,
                            ack.result
                        );

                        // this case is when its ack for different command
                        if (ack.command != 400) {
                            continue;
                        }

                        if (ack.result == 0) {
                            printf(LOG_STATE " Vehicle armed\n");
                            // state = CONTROL_HOLD_POSITION;
                            printf(LOG_STATE " Replay started: samples=%zu rate=10Hz\n",
                                    resample.count);
                            state = CONTROL_REPLAY_TRAJECTORY;
                        } else {
                            fprintf(stderr, LOG_ERROR " Vehicle arm rejected: result=%u\n",
                                    ack.result);
                            state = CONTROL_ERROR;
                        }
                    }
                }

                if (state == CONTROL_WAIT_ARM_ACK
                        && monotonic_time_ms() - arm_last_send_ms
                            >= command_retry_interval_ms) {
                    if (arm_attempts >= command_max_attempts) {
                        fprintf(stderr, LOG_ERROR " Arm acknowledgement timed out: attempts=%zu\n",
                                arm_attempts);
                        state = CONTROL_ERROR;
                        break;
                    }

                    MAVLinkEncodedFrame_t arm_command;

                    if (!mavlink_encode_arm(&arm_command,
                                source_system, source_component,
                                target_system, target_component)) {
                        fprintf(stderr, LOG_ERROR " Failed to encode Arm command retry\n");
                        state = CONTROL_ERROR;
                        break;
                    }

                    ssize_t sent = sendto(fd, arm_command.bytes,
                            arm_command.length, 0,
                            (const struct sockaddr *)&px4_address, px4_address_length);

                    if (sent != (ssize_t)arm_command.length) {
                        perror(LOG_ERROR " Failed to resend Arm command");
                        state = CONTROL_ERROR;
                        break;
                    }

                    arm_attempts++;
                    arm_last_send_ms = monotonic_time_ms();
                    printf(LOG_STATE " Vehicle arm requested: attempt=%zu/%zu\n",
                            arm_attempts, command_max_attempts);
                }

                break;
            }

            case CONTROL_HOLD_POSITION: {
                if (!setpoint_due) {
                    break;
                }

                if (input_ready) {
                    char input[16];
                    ssize_t count = read(STDIN_FILENO, input, sizeof(input));

                    if (count > 0 && input[0] == 'l') {
                        printf("[INPUT] Landing requested\n");
                        state = CONTROL_LAND;
                        break;
                    }
                }

                MAVLinkEncodedFrame_t command;

                if (has_last_commanded_position) {
                    if (!mavlink_encode_position_target_local_ned(&command, source_system, source_component, 
                                target_system, target_component, 0, 
                                last_commanded_position.x, last_commanded_position.y, last_commanded_position.z)) {
                        state = CONTROL_ERROR;
                        break;
                    }
                } else {
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
                if (!setpoint_due) {
                    break;
                }
                
                if (replay_index >= resample.count) {
                    printf(LOG_STATE " Replay complete: sent=%zu; holding last commanded position\n",
                            resample.count);
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
                        "[REPLAY] sample=%zu/%zu time=%.1fs target_ned=(%+.3f, %+.3f, %+.3f)\n",
                        replay_index + 1,
                        resample.count,
                        (double)position->elapsed_us / 1000000.0,
                        position->x,
                        position->y,
                        position->z
                    );
                }

                // this is going to be used in the CONTORL_HOLD_POSITION
                last_commanded_position = resample.positions[replay_index];
                has_last_commanded_position = true;

                replay_index++;
                break;
            }

            case CONTROL_LAND: {
                break;
            }

            case CONTROL_ERROR:
                break;
        }

        if (setpoint_due) {
            now_ms = monotonic_time_ms();

            do {
                next_setpoint_ms += 100;
            } while (next_setpoint_ms <= now_ms);
        }
    }


    flight_data_free(&flight_data);
    replay_trajectory_free(&trajectory);
    replay_trajectory_free(&resample);

    return 0;
}


/* ---------- function declarations ---------- */

static uint64_t monotonic_time_ms(void) {
    struct timespec now = {0};
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (uint64_t)now.tv_sec * UINT64_C(1000)
        + (uint64_t)now.tv_nsec / UINT64_C(1000000);
}

static bool send_position_setpoint(
        int fd, const struct sockaddr_in *px4_address,
        socklen_t px4_address_length,
        uint8_t source_system, uint8_t source_component,
        uint8_t target_system, uint8_t target_component,
        float x, float y, float z) {
    MAVLinkEncodedFrame_t command;

    if (!mavlink_encode_position_target_local_ned(
                &command,
                source_system, source_component,
                target_system, target_component,
                0, x, y, z)) {
        return false;
    }

    ssize_t sent = sendto(fd, command.bytes, command.length, 0,
            (const struct sockaddr *)px4_address, px4_address_length);

    return (sent == (ssize_t)command.length);
}

static bool drain_udp_datagrams(int fd) {
    uint8_t discard_buffer[2048];

    while (true) {
        ssize_t received = recvfrom(
            fd, discard_buffer, sizeof(discard_buffer), MSG_DONTWAIT,
            NULL, NULL
        );

        if (received >= 0) {
            continue;
        }

        if (errno == EINTR) {
            continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return true;
        }

        perror(LOG_ERROR " Failed to receive UDP datagram");
        return false;
    }
}
