#include <stdio.h>
#include <stdbool.h>

#include "../../include/tlog.h"
#include "../../include/flight_data.h"
#include "../../include/mavlink_types.h"
#include "../../include/mavlink_decode.h"

bool validator_handle_frame(TLogRecord_t *tlog, FlightData_t *flight_data, uint32_t msgid) {
    switch(msgid) {
        case 0: {
            MAVLinkHeartbeat_t heartbeat;

            if (!mavlink_decode_heartbeat(&tlog->frame, &heartbeat)) {
                return false;
            }

            return flight_data_add_heartbeat(flight_data, tlog->timestamp_us, &heartbeat);
        }

        case 30: {
            MAVLinkAttitude_t attitude;

            if (!mavlink_decode_attitude(&tlog->frame, &attitude)) {
                return false;
            }

            return flight_data_add_attitude(flight_data, tlog->timestamp_us, &attitude);
        }

        case 31: {
            MAVLinkAttitudeQuaternion_t attitude;

            if (!mavlink_decode_attitudeQuaternion(&tlog->frame, &attitude)) {
                return false;
            }

            return flight_data_add_attitude_quaternion(flight_data, tlog->timestamp_us, &attitude);
        }

        case 32: {
            MAVLinkLocalPositionNed_t position;

            if (!mavlink_decode_localPositionNed(&tlog->frame, &position)) {
                return false;
            }

            return flight_data_add_local_position(flight_data, tlog->timestamp_us, &position);
        }

        case 33: {
            MAVLinkGlobalPositionInt_t position;

            if (!mavlink_decode_globalPositionInt(&tlog->frame, &position)) {
                return false;
            }

            return flight_data_add_global_position(flight_data, tlog->timestamp_us, &position);
        }

        case 84: {
            MAVLinkSetPositionTargetLocalNed_t target;

            if (!mavlink_decode_setPositionTargetLocalNed(&tlog->frame, &target)) {
                return false;
            }

            return flight_data_add_set_position_target(flight_data, tlog->timestamp_us, &target);
        }

        case 85: {
            MAVLinkPositionTargetLocalNed_t target;

            if (!mavlink_decode_positionTargetLocalNed(&tlog->frame, &target)) {
                return false;
            }

            return flight_data_add_position_target(flight_data, tlog->timestamp_us, &target);
        }

        default:
            return true;
    }
    
    return true;
}


void print_read_summary(FlightData_t *flight_data, size_t records_read, size_t valid_frames, size_t invalid_crc, size_t unknown_crc) {

    size_t decoded_total =
        flight_data->heartbeat_count +
        flight_data->attitude_count +
        flight_data->attitude_quaternion_count +
        flight_data->local_position_count +
        flight_data->global_position_count +
        flight_data->set_position_target_count +
        flight_data->position_target_count;

    printf("\n");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10s |\n", "Validation summary", "Count");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10zu |\n", "Records read", records_read);
    printf("| %-28s | %10zu |\n", "Valid frames", valid_frames);
    printf("| %-28s | %10zu |\n", "Invalid CRC", invalid_crc);
    printf("| %-28s | %10zu |\n", "Unknown CRC", unknown_crc);
    printf("+------------------------------+------------+\n");

    printf("\n");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10s |\n", "Decoded messages", "Count");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10zu |\n", "Heartbeat (0)", flight_data->heartbeat_count);
    printf("| %-28s | %10zu |\n", "Attitude (30)", flight_data->attitude_count);
    printf("| %-28s | %10zu |\n", "Attitude quaternion (31)", flight_data->attitude_quaternion_count);
    printf("| %-28s | %10zu |\n", "Local position NED (32)", flight_data->local_position_count);
    printf("| %-28s | %10zu |\n", "Global position INT (33)", flight_data->global_position_count);
    printf("| %-28s | %10zu |\n", "Set position target (84)", flight_data->set_position_target_count);
    printf("| %-28s | %10zu |\n", "Position target (85)", flight_data->position_target_count);
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10zu |\n", "Total", decoded_total);
    printf("+------------------------------+------------+\n");
}
