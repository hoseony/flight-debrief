#include <stdio.h>
#include <stdbool.h>

#include "../../include/tlog.h"
#include "../../include/mavlink_types.h"
#include "../../include/flight_data.h"
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
