#include <stdbool.h>
#include <stdio.h>

#include "../../include/mavlink_decode.h"
#include "../../include/mavlink_types.h"
#include "../../include/log.h"

void mavlink_handle_frame(const MAVLinkFrame_t *frame, const MAVLinkLogs_t *logs, uint32_t msgid){
    /* log it straight into the .tlog first */ 


    switch(msgid) {
        case 0: {
            MAVLinkHeartbeat_t heartbeat;

            if (mavlink_decode_heartbeat(frame, &heartbeat)) {
                printf("HEARTBEAT: mode=%u status=%u\n", heartbeat.base_mode, heartbeat.system_status);
            }

            break;
        }

        case 30: {
            MAVLinkAttitude_t attitude; 

            if (mavlink_decode_attitude(frame, &attitude)) {
                log_write_attitude(logs->attitude, &attitude);
            }

            break;
        }

        case 31: {
            MAVLinkAttitudeQuaternion_t attitude; 

            if (mavlink_decode_attitudeQuaternion(frame, &attitude)) {
                log_write_attitudeQuaternion(logs->attitude_quaternion, &attitude);
            }

            break;
        }

        case 32: {
            MAVLinkLocalPositionNed_t position;

            if (mavlink_decode_localPositionNed(frame, &position)) {
                log_write_localPositionNed(logs->local_position, &position);
            }

            break;
        }

        case 33: {
            MAVLinkGlobalPositionInt_t position;

            if (mavlink_decode_globalPositionInt(frame, &position)) {
                log_write_globalPositionInt(logs->global_position, &position);
            }
            
            break;
        }

        case 85: {
            MAVLinkPositionTargetLocalNed_t target;

            if (mavlink_decode_positionTargetLocalNed(frame, &target)) {
                log_write_positionTargetLocalNed(logs->position_target, &target);
            }
            
            break;
        }

        default:
            break;
    }

}
