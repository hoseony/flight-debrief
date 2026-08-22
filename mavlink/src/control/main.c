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
 *      a. transport data arrives
 *         -> wake, receive available bytes
 *         -> parse Heartbeat or command_ack
 *         -> change the control state if necessary
 *      b. next 10hz deadline arrives
 *         -> send the setpoint 
 *         -> move the deadline forward 100ms
 * 4. rest is state machine
 *
 * Overall Flow: 
 *      Load trajectory          -> Wait for Heartbeat
 *      -> Capture local origin  -> Prestream setpoints
 *      -> Request Offboard
 *      -> Wait for Offboard ACK -> Request Arm
 *      -> Wait for Arm ACK      -> Replay trajectory at 10 Hz
 *      -> Hold final position   -> Land (sequence)
 */

/* ---------- #include / #define ---------- */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <poll.h>

#include "../../include/mavlink_decode.h"
#include "../../include/mavlink_parser.h"
#include "../../include/control_transport.h"
#include "../../include/mavlink_encoder.h"
#include "../../include/replay.h"
#include "../../include/px4_messages.h"
#include "../../include/px4_commands.h"
#include "../../include/control_replay.h"

#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_CYAN "\033[36m"
#define COLOR_RESET "\033[0m"

#define LOG_ERROR COLOR_RED "[ERROR]" COLOR_RESET
#define LOG_ACK COLOR_GREEN "[ACK]" COLOR_RESET
#define LOG_TRACK COLOR_YELLOW "[TRACK]" COLOR_RESET
#define LOG_STATE COLOR_CYAN "[STATE]" COLOR_RESET

/* ---------- function prototype ---------- */
static uint64_t monotonic_time_ms(void);

/* ---------- FSM (states) ---------- */
// FSM approach makes it a bit cleaner (believe or not...)
typedef enum {
    CONTROL_WAIT_HEARTBEAT,
    CONTROL_WAIT_LOCAL_POSITION,
    CONTROL_PRESTREAM_SETPOINT,
    CONTROL_WAIT_OFFBOARD_ACK,
    CONTROL_WAIT_ARM_ACK,
    CONTROL_HOLD_POSITION,
    CONTROL_REPLAY_TRAJECTORY,
    CONTROL_COMMAND_LAND,
    CONTROL_WAIT_LAND_ACK,
    CONTROL_WAIT_DISARMED,
    CONTROL_COMPLETE,
    CONTROL_ERROR
} ControlState_t;

/* ---------- main function ---------- */
int main(int argc, char **argv) {

    bool serial_mode = argc >= 2 && strcmp(argv[1], "serial") == 0;
    bool valid_arguments = serial_mode
        ? (argc == 4 || (argc == 5 && strcmp(argv[4], "--diff") == 0))
        : (argc == 2 || (argc == 3 && strcmp(argv[2], "--diff") == 0));

    if (!valid_arguments) {
        fprintf(stderr,
                "Usage: %s <telemetry.tlog> [--diff]\n"
                "       %s serial <device> <telemetry.tlog> [--diff]\n",
                argv[0], argv[0]);
        return 1;
    }

    const char *serial_device = serial_mode ? argv[2] : NULL;
    const char *trajectory_path = serial_mode ? argv[3] : argv[1];
    bool tracking_diff_enabled = serial_mode ? argc == 5 : argc == 3;

    ReplayTrajectory_t resample = {0};
    if (!prepare_replay_trajectory(trajectory_path, &resample)) {
        return 1;
    }

    const ReplayPosition_t *initial_position = &resample.positions[0];

    /* ---------- open transport ---------- */
    ControlTransport_t transport;
    bool transport_opened = serial_mode
        ? control_transport_open_serial(&transport, serial_device)
        : control_transport_open_udp(&transport, 14550);

    if (!transport_opened) {
        perror(LOG_ERROR " Failed to open control transport");
        replay_trajectory_free(&resample);
        return 1;
    }

    /* ---------- FSM starts here ---------- */
    ControlState_t state = CONTROL_WAIT_HEARTBEAT;

    /* state: CONTROL_WAIT_HEARTBEAT */
    uint8_t buffer[2048];
    MAVLinkParser_t parser;
    MAVLinkFrame_t frame;
    mavlink_parser_init(&parser);

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
    const uint64_t disarm_wait_timeout_ms = 30000;

    // Every time it receives heartbeat message 
    // it updates "last_heartbeat_ms" to be the current time.
    // if the difference between last_heartbeat_ms and current time
    // exceeds 3000ms (3s), it assumes the connection is lost
    const uint64_t heartbeat_timeout_ms = 3000;

    uint64_t offboard_last_send_ms = 0;
    uint64_t arm_last_send_ms = 0;
    uint64_t land_last_send_ms = 0;
    uint64_t disarm_wait_start_ms = 0;
    size_t offboard_attempts = 0;
    size_t arm_attempts = 0;
    size_t land_attempts = 0;
    uint64_t last_heartbeat_ms = 0;
    uint64_t next_tracking_log_ms = 0;

    /* command last pos */
    ReplayPosition_t last_commanded_position;
    bool has_last_commanded_position = false;
    uint64_t next_setpoint_ms = monotonic_time_ms() + 100;



    printf(LOG_STATE " Waiting for PX4 heartbeat\n");

    /* ---------- MAIN LOOP ---------- */
    while (state != CONTROL_ERROR && state != CONTROL_COMPLETE) {
        uint64_t now_ms = monotonic_time_ms();
        int timeout_ms = -1;

        // Wake periodically while waiting for the local position. Other
        // active states wake on the next 10 Hz setpoint deadline.
        if (state == CONTROL_WAIT_LOCAL_POSITION) {
            timeout_ms = 1000;
        } else if (state != CONTROL_WAIT_HEARTBEAT) {
            timeout_ms = (next_setpoint_ms > now_ms) ? (int)(next_setpoint_ms - now_ms) : 0;
        }

        struct pollfd events[2]= {
            {
                .fd = control_transport_fd(&transport),
                .events = POLLIN
            },
            {
                .fd = STDIN_FILENO, // macro for stdin fd
                .events = POLLIN
            }
        };
        
        // poll is conditional timeout 
        // thiscase when the "timeout" is passed or "POLLIN"
        int poll_result = poll(events, 2, timeout_ms);

        // error handling
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }

            perror(LOG_ERROR " poll failed");
            state = CONTROL_ERROR;
            break;
        }

        // catch transport event error
        if (events[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fprintf(stderr, LOG_ERROR " Control transport event failure: revents=0x%x\n",
                    events[0].revents);
            state = CONTROL_ERROR;
            break;
        }

        if (events[1].revents & POLLNVAL) {
            fprintf(stderr, LOG_ERROR " Standard input is unavailable\n");
            state = CONTROL_ERROR;
            break;
        }

        // This is when you can read the transport.
        // It ensures there exists some data
        bool transport_ready = (poll_result > 0) && (events[0].revents & POLLIN);
        bool input_ready = (poll_result > 0) && (events[1].revents & POLLIN);
        // update now
        now_ms = monotonic_time_ms();

        // condition that checks if it needs to send setpoint signal
        bool setpoint_due = state != CONTROL_WAIT_HEARTBEAT
            && state != CONTROL_WAIT_LOCAL_POSITION
            && now_ms >= next_setpoint_ms;

        // This is where all the necessary messages are being captured
        PX4ReceivedMessages_t received_messages = {0};

        if (transport_ready && state != CONTROL_WAIT_HEARTBEAT) {
            if (!receive_px4_messages(
                        &transport, &parser, &received_messages)) {
                state = CONTROL_ERROR;
                break;
            }
        }

        // if the message received is heartbeat, update the last_heartbeat_ms
        if (received_messages.heartbeat_received) {
            last_heartbeat_ms = now_ms;
        }

        // if the message received is ACK, print which ack type was received
        if (received_messages.command_ack_received) {
            printf(LOG_ACK " command=%u result=%u\n",
                    received_messages.command_ack.command,
                    received_messages.command_ack.result);
        }

        if (tracking_diff_enabled
                && received_messages.local_position_received
                && has_last_commanded_position
                && now_ms >= next_tracking_log_ms) {
            const MAVLinkLocalPositionNed_t *actual =
                &received_messages.local_position;
            float error_x = actual->x - last_commanded_position.x;
            float error_y = actual->y - last_commanded_position.y;
            float error_z = actual->z - last_commanded_position.z;
            float error_3d = hypotf(hypotf(error_x, error_y), error_z);

            printf(LOG_TRACK " target=(%+.3f, %+.3f, %+.3f) "
                    "actual=(%+.3f, %+.3f, %+.3f) error=%.3fm\n",
                    last_commanded_position.x,
                    last_commanded_position.y,
                    last_commanded_position.z,
                    actual->x, actual->y, actual->z,
                    error_3d);
            next_tracking_log_ms = now_ms + UINT64_C(1000);
        }

        // heartbeat failed! It fucking died!!!
        if (px4_found
                && state != CONTROL_WAIT_HEARTBEAT
                && now_ms - last_heartbeat_ms >= heartbeat_timeout_ms) {
            fprintf(stderr, LOG_ERROR " PX4 connection lost: no heartbeat for %llums\n",
                    (unsigned long long)heartbeat_timeout_ms);
            state = CONTROL_ERROR;
            break;
        }

        switch(state) {
            // waiting for the heartbeat from the drone 
            // to check the address it is coming from
            // once it finds the drone, it proceeds to next state
            case CONTROL_WAIT_HEARTBEAT: {
                if (!transport_ready) {
                    break;
                }

                ssize_t received = control_transport_receive(
                        &transport, buffer, sizeof(buffer));

                if (received < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        break;
                    }
                    
                    perror(LOG_ERROR " Failed to receive control data");
                    state = CONTROL_ERROR;
                    break;
                }

                if (received == 0) {
                    break;
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

                    control_transport_accept_peer(&transport);
                    target_system = frame.bytes[5];
                    target_component = frame.bytes[6];
                    px4_found = true;
                    last_heartbeat_ms = monotonic_time_ms();

                    char connection[160];
                    control_transport_describe_peer(
                            &transport, connection, sizeof(connection));

                    printf(
                        "[INFO] PX4 discovered: connection=%s system=%u component=%u\n",
                        connection,
                        frame.bytes[5],
                        frame.bytes[6]
                    );
                }

                // we found it!
                if (px4_found) {
                    printf(LOG_STATE " Waiting for current local position\n");
                    state = CONTROL_WAIT_LOCAL_POSITION;
                }

                break;
            }

            case CONTROL_WAIT_LOCAL_POSITION: {
                if (!received_messages.local_position_received) {
                    break;
                }

                const MAVLinkLocalPositionNed_t *origin =
                    &received_messages.local_position;

                if (!control_replay_apply_origin(
                            &resample, origin->x, origin->y, origin->z)) {
                    fprintf(stderr, LOG_ERROR " Invalid replay origin\n");
                    state = CONTROL_ERROR;
                    break;
                }

                printf("[INFO] Replay origin: ned=(%+.3f, %+.3f, %+.3f)\n",
                        origin->x, origin->y, origin->z);
                printf(LOG_STATE " Prestreaming setpoints: count=20 rate=10Hz\n");
                next_setpoint_ms = monotonic_time_ms();
                state = CONTROL_PRESTREAM_SETPOINT;
                break;
            }

            // Before sending offboard_command,
            // It needs to receive prestreaming for about 1s
            case CONTROL_PRESTREAM_SETPOINT: {
                if (!setpoint_due) {
                    break;
                }

                if (!send_position_setpoint(
                            &transport,
                            source_system, source_component,
                            target_system, target_component,
                            initial_position->x,
                            initial_position->y,
                            initial_position->z)) {
                    state = CONTROL_ERROR;
                    break;
                }

                setpoint_count++;

                // after the prestreaming, send offboard_signal
                if (setpoint_count >= 20) {
                    printf("[INFO] Prestream complete: sent=%zu\n", setpoint_count);

                    if (!encode_and_send_command(
                                &transport,
                                source_system, source_component,
                                target_system, target_component,
                                mavlink_encode_set_offboard_mode)) {
                        fprintf(stderr,
                                LOG_ERROR " Failed to encode or send Offboard command\n");
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
                            &transport,
                            source_system, source_component,
                            target_system, target_component,
                            initial_position->x,
                            initial_position->y,
                            initial_position->z)) {
                    state = CONTROL_ERROR;
                    break;
                }

                if (received_messages.command_ack_received
                        && received_messages.command_ack.command == 176) {
                    MAVLinkCommandAck_t *ack = &received_messages.command_ack;

                    if (ack->result == 0) {
                        printf(LOG_STATE " Offboard mode accepted\n");

                        if (!encode_and_send_command(
                                    &transport,
                                    source_system, source_component,
                                    target_system, target_component,
                                    mavlink_encode_arm)) {
                            fprintf(stderr,
                                    LOG_ERROR " Failed to encode or send Arm command\n");
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
                                ack->result);
                        state = CONTROL_ERROR;
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

                    if (!encode_and_send_command(
                                &transport,
                                source_system, source_component,
                                target_system, target_component,
                                mavlink_encode_set_offboard_mode)) {
                        fprintf(stderr,
                                LOG_ERROR " Failed to encode or resend Offboard command\n");
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
                            &transport,
                            source_system, source_component,
                            target_system, target_component,
                            initial_position->x,
                            initial_position->y,
                            initial_position->z)) {
                    state = CONTROL_ERROR;
                    break;
                }

                if (received_messages.command_ack_received
                        && received_messages.command_ack.command == 400) {
                    MAVLinkCommandAck_t *ack = &received_messages.command_ack;

                    if (ack->result == 0) {
                        printf(LOG_STATE " Vehicle armed\n");
                        printf(LOG_STATE " Replay started: samples=%zu rate=10Hz\n",
                                resample.count);
                        state = CONTROL_REPLAY_TRAJECTORY;
                    } else {
                        fprintf(stderr, LOG_ERROR " Vehicle arm rejected: result=%u\n",
                                ack->result);
                        state = CONTROL_ERROR;
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

                    if (!encode_and_send_command(
                                &transport,
                                source_system, source_component,
                                target_system, target_component,
                                mavlink_encode_arm)) {
                        fprintf(stderr,
                                LOG_ERROR " Failed to encode or resend Arm command\n");
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
                        state = CONTROL_COMMAND_LAND;
                        break;
                    }
                }

                if (!has_last_commanded_position) {
                    state = CONTROL_ERROR;
                    break;
                }

                if (!send_position_setpoint(
                            &transport,
                            source_system, source_component,
                            target_system, target_component,
                            last_commanded_position.x,
                            last_commanded_position.y,
                            last_commanded_position.z)) {
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

                if (!send_position_setpoint(
                            &transport,
                            source_system, source_component,
                            target_system, target_component,
                            position->x, position->y, position->z)) {
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

            case CONTROL_COMMAND_LAND: {
                if (!encode_and_send_command(
                            &transport,
                            source_system, source_component,
                            target_system, target_component,
                            mavlink_encode_land)) {
                    fprintf(stderr,
                            LOG_ERROR " Failed to encode or send Land command\n");
                    state = CONTROL_ERROR;
                    break;
                }

                land_attempts = 1;
                land_last_send_ms = monotonic_time_ms();
                printf(LOG_STATE " Landing requested: attempt=1/%zu\n",
                        command_max_attempts);
                printf(LOG_STATE " Waiting for Land acknowledgement\n");
                state = CONTROL_WAIT_LAND_ACK;
                break;
            }

            case CONTROL_WAIT_LAND_ACK: {
                if (received_messages.command_ack_received
                        && received_messages.command_ack.command == 21) {
                    MAVLinkCommandAck_t *ack = &received_messages.command_ack;

                    if (ack->result == 0) {
                        printf(LOG_STATE " Land command accepted\n");
                        printf(LOG_STATE " Waiting for automatic disarm\n");
                        disarm_wait_start_ms = monotonic_time_ms();
                        state = CONTROL_WAIT_DISARMED;
                    } else {
                        fprintf(stderr,
                                LOG_ERROR " Land command rejected: result=%u\n",
                                ack->result);
                        state = CONTROL_ERROR;
                    }
                }

                if (state == CONTROL_WAIT_LAND_ACK
                        && monotonic_time_ms() - land_last_send_ms
                            >= command_retry_interval_ms) {
                    if (land_attempts >= command_max_attempts) {
                        fprintf(stderr,
                                LOG_ERROR " Land acknowledgement timed out: attempts=%zu\n",
                                land_attempts);
                        state = CONTROL_ERROR;
                        break;
                    }

                    if (!encode_and_send_command(
                                &transport,
                                source_system, source_component,
                                target_system, target_component,
                                mavlink_encode_land)) {
                        fprintf(stderr,
                                LOG_ERROR " Failed to encode or resend Land command\n");
                        state = CONTROL_ERROR;
                        break;
                    }

                    land_attempts++;
                    land_last_send_ms = monotonic_time_ms();
                    printf(LOG_STATE " Landing requested: attempt=%zu/%zu\n",
                            land_attempts, command_max_attempts);
                }

                break;
            }

            case CONTROL_WAIT_DISARMED: {
                const uint8_t armed_flag = UINT8_C(0x80);

                if (received_messages.heartbeat_received
                        && (received_messages.heartbeat.base_mode & armed_flag) == 0) {
                    printf(LOG_STATE " Vehicle disarmed\n");
                    state = CONTROL_COMPLETE;
                }

                if (state == CONTROL_WAIT_DISARMED
                        && monotonic_time_ms() - disarm_wait_start_ms
                            >= disarm_wait_timeout_ms) {
                    fprintf(stderr,
                            LOG_ERROR " Automatic disarm timed out after %llums\n",
                            (unsigned long long)disarm_wait_timeout_ms);
                    state = CONTROL_ERROR;
                }

                break;
            }

            case CONTROL_COMPLETE:
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


    replay_trajectory_free(&resample);

    if (!control_transport_close(&transport)) {
        perror(LOG_ERROR " Failed to close control transport");
    }

    return (state == CONTROL_COMPLETE) ? 0 : 1;
}


/* ---------- function declarations ---------- */

static uint64_t monotonic_time_ms(void) {
    struct timespec now = {0};
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (uint64_t)now.tv_sec * UINT64_C(1000)
        + (uint64_t)now.tv_nsec / UINT64_C(1000000);
}
