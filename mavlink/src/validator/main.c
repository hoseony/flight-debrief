// [ Some note about ../validator ]
//
// The pi will only record the .tlog file while flying (that's the recorder part).
// I did this to avoid assigning too much task to the pi, as I wasn't confident about 
// it's computing power. Then everything else will be done after the recording or the flight.
// 
// This is one of the post-process related program.
// It will decode the stored .tlog file (path read from the command line argument)
// and store the ones with valid crc and known message type for this project
// as limited amount of messages were implemented.

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "../../include/mavlink_types.h"
#include "../../include/mavlink_decode.h"
#include "../../include/flight_data.h"
#include "../../include/tlog.h"

int main(int argc, char* argv[]) {

    /* opening the file from the command line arg */
    if (argc != 2) {
        fprintf(stderr, "usage: %s <telemetry.tlog>\n", argv[0]);
        return 1;
    }

    FILE *file = fopen(argv[1], "rb");

    if (file == NULL) {
        fprintf(stderr, "cannot open %s: %s\n", argv[1], strerror(errno));
        return 1;
    }

    /* maing validation loop */
    // the loop breaks when there no more things to read
    // (count == 0 && feof(file))

    FlightData_t flight_data = {0};
    printf("READING %s\n", argv[1]);

    while (true) {
        TLogRecord_t tlog = {0};

        /* ---------- read timestamp ---------- */
        uint8_t timestamp_bytes[8];
        size_t count = fread(timestamp_bytes, 1, 8, file);

        // no more data exist
        if (count == 0 && feof(file)) {
            break;
        }

        // uhh that's not good
        if (count != 8) {
            fprintf(stderr, "truncated timestamp\n");
            break;
        }

        for (size_t i = 0; i < 8; i++) {
            tlog.timestamp_us = (tlog.timestamp_us << 8) | timestamp_bytes[i];
        }

        printf("\n[%" PRIu64 "]\n", tlog.timestamp_us);


        /* ---------- reconstruct frame ---------- */
        if (fread(tlog.frame.bytes, 1, 3, file) != 3) {
            // do something? idk this an error
            return 1;
        }
        
        size_t frame_length = 10 + tlog.frame.bytes[1] + 2;
        if (tlog.frame.bytes[2] & 0x01) {
            frame_length += 13;
        }

        size_t remaining = frame_length - 3u;
        if (fread(&tlog.frame.bytes[3], 1, remaining, file) != remaining) {
            // same case as above, this also an error, couldn't read enough
            return 1;
        }

        tlog.frame.length = frame_length;
        
        for (size_t i = 0; i < frame_length; i++) {
            printf("%02X", tlog.frame.bytes[i]);

            if ( ((i + 1) % 8) == 0) {
                putchar('\n');
            }
        }

        if (frame_length % 8 != 0) {
            putchar('\n');
        }

        /* ---------- crc validation here ----------*/
        uint32_t msgid = frame_msgid(&tlog.frame);
        uint8_t crc_extra; 

        // get crc_extra
        if(!mavlink_crc_extra_for(msgid, &crc_extra)) {
            continue;
        }
        
        // validate crc
        // msg with unknown crcs will get skipped here
        if (!mavlink_frame_crc_valid(&tlog.frame, crc_extra)) {
            fprintf(stderr, "invalid crc for message %u\n", msgid);
            continue;
        }

        putchar('\n');

        /* actual decoding happens here */
        // YES, I WILL MOVE THIS TO ANOTHER FNCTION LATER
        switch(msgid) {
            case 0: {
                MAVLinkHeartbeat_t heartbeat;

                if (!mavlink_decode_heartbeat(&tlog.frame, &heartbeat)) {
                    fprintf(stderr, "failed to decode heartbeat\n");
                    break;
                }

                if (!flight_data_add_heartbeat(&flight_data, tlog.timestamp_us, &heartbeat)) {
                    fprintf(stderr, "failed to store heartbeat\n");
                }

                break;
            }

            case 30: {
                MAVLinkAttitude_t attitude;

                if (!mavlink_decode_attitude(&tlog.frame, &attitude)) {
                    fprintf(stderr, "failed to decode attitude\n");
                    break;
                }

                if (!flight_data_add_attitude(&flight_data, tlog.timestamp_us, &attitude)) {
                    fprintf(stderr, "failed to store attitude\n");
                }

                break;
            }

            case 31: {
                MAVLinkAttitudeQuaternion_t attitude;

                if (!mavlink_decode_attitudeQuaternion(&tlog.frame, &attitude)) {
                    fprintf(stderr, "failed to decode attitude quaternion\n");
                    break;
                }

                if (!flight_data_add_attitude_quaternion(&flight_data, tlog.timestamp_us, &attitude)) {
                    fprintf(stderr, "failed to store attitude quaternion\n");
                }

                break;
            }

            case 32: {
                MAVLinkLocalPositionNed_t position;

                if (!mavlink_decode_localPositionNed(&tlog.frame, &position)) {
                    fprintf(stderr, "failed to decode local position\n");
                    break;
                }

                if (!flight_data_add_local_position(&flight_data, tlog.timestamp_us, &position)) {
                    fprintf(stderr, "failed to store local position\n");
                }

                break;
            }

            case 33: {
                MAVLinkGlobalPositionInt_t position;

                if (!mavlink_decode_globalPositionInt(&tlog.frame, &position)) {
                    fprintf(stderr, "failed to decode global position\n");
                    break;
                }

                if (!flight_data_add_global_position(&flight_data, tlog.timestamp_us, &position)) {
                    fprintf(stderr, "failed to store global position\n");
                }

                break;
            }

            case 85: {
                MAVLinkPositionTargetLocalNed_t target;

                if (!mavlink_decode_positionTargetLocalNed(&tlog.frame, &target)) {
                    fprintf(stderr, "failed to decode position target\n");
                    break;
                }

                if (!flight_data_add_position_target(&flight_data, tlog.timestamp_us, &target)) {
                    fprintf(stderr, "failed to store position target\n");
                }

                break;
            }

            default:
                break;
        }
    }

    flight_data_free(&flight_data);


    /* close the file */
    if (fclose(file) == EOF) {
        perror("fclose");
        return 1;
    }

    return 0;
}
