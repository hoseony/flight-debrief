#include <stdbool.h>
#include <stdio.h>

#include "../include/mavlink_decode.h"
#include "../include/mavlink_types.h"

void mavlink_handle_frame(const MAVLinkFrame_t *frame, FILE *file, uint32_t msgid){
    
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

            }

            break;
        }

        case 31: {
            MAVLinkAttitudeQuaternion_t attitude; 

            if (mavlink_decode_attitudeQuaternion(frame, &attitude)) {

            }

            break;
        }

        case 32: {
            MAVLinkLocalPositionNed_t position;

            if (mavlink_decode_localPositionNed(frame, &position)) {

            }

            break;
        }

        case 33: {
            MAVLinkGlobalPositionInt_t position;

            if (mavlink_decode_globalPositionInt(frame, &position)) {

            }
            
            break;
        }

        case 85: {
            MAVLinkPositionTargetLocalNed_t target;

            if (mavlink_decode_positionTargetLocalNed(frame, &target)) {

            }
            
            break;
        }

        default:
            break;
    }

}
