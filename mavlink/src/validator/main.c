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
#include <stdbool.h>

#include "../../include/mavlink_types.h"
#include "../../include/mavlink_decode.h"
#include "../../include/flight_data.h"
#include "../../include/tlog.h"
#include "../../include/validator.h"

int main(int argc, char* argv[]) {
    int exit_status = 0;

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

    /* ---------- main validator loop ---------- */
    FlightData_t flight_data = {0};
    printf("READING %s\n", argv[1]);

    size_t records_read = 0;
    size_t valid_frames = 0;
    size_t invalid_crc = 0;
    size_t unknown_crc = 0;

    while (true) {

        /* ---------- read timestamp and MAVLink frame ---------- */
        TLogRecord_t tlog = {0};
        TLogReadResult_t read_result = tlog_read(file, &tlog);

        if (read_result == TLOG_READ_EOF) {
            break;
        }

        if (read_result == TLOG_READ_ERROR) {
            fprintf(stderr, "failed to read tlog...\n");
            exit_status = 1;
            break;
        }

        records_read++; 

        /* ---------- crc validation here ----------*/
        uint32_t msgid = frame_msgid(&tlog.frame);
        uint8_t crc_extra; 

        // get crc_extra
        if(!mavlink_crc_extra_for(msgid, &crc_extra)) {
            unknown_crc++;
            continue;
        }
        
        // validate crc
        // msg with unknown crcs will get skipped here
        if (!mavlink_frame_crc_valid(&tlog.frame, crc_extra)) {
            fprintf(stderr, "invalid crc for message %u\n", msgid);
            invalid_crc++;
            continue;
        }

        valid_frames++;

        /* ---------- decoding / storing ----------*/
        if (!validator_handle_frame(&tlog, &flight_data, msgid)) {
            fprintf(stderr, "failed to decode or store message %u\n", msgid);
        }

    }
    print_read_summary(&flight_data, records_read, valid_frames, invalid_crc, unknown_crc);
    flight_data_free(&flight_data);

    /* close the file */
    if (fclose(file) == EOF) {
        perror("fclose");
        return 1;
    }

    return exit_status;
}
