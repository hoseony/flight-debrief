#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>

#include "../../include/mavlink_types.h"
#include "../../include/mavlink_decode.h"
#include "../../include/flight_data.h"

bool flight_data_add_attitude(FlightData_t *flight_data, uint64_t timestamp_us, MAVLinkAttitude_t *attitude);
void flight_data_free(FlightData_t *flight_data);

// The pi will only record the .tlog file while flying. 
// I did this to avoid doing too much processing within the pi itself.
// Then everything else will be done after the recording (after the flight).
// let's decode the stored .tlog file...

// this struct will be used to completely decode the .tlog file 
// .tlog file have 8bit timestamp and then the frame
typedef struct {
    uint64_t timestamp_us;
    MAVLinkFrame_t frame;
} TLogRecord_t;

// This project has limited number of crc support for now.
// But, it should be able to tell if the frame is valid, invalid,
// or unspported. Then only save correct frame into....

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

    FlightData_t flight_data = {0};

    /* maing validating loop */
    for (int j = 0; j < 2; j++) {
        TLogRecord_t tlog = {0};

        /* read timestamp */
        uint8_t timestamp_bytes[8];
        size_t count = fread(timestamp_bytes, 1, 8, file);

        // no more data exist
        if (count == 0 && feof(file)) {
            break;
        }

        if (count != 8) {
            fprintf(stderr, "truncated timestamp\n");
        }

        for (size_t i = 0; i < 8; i++) {
            tlog.timestamp_us = (tlog.timestamp_us << 8) | timestamp_bytes[i];
        }

        printf("%" PRIu64 "\n", tlog.timestamp_us);

        /* reconstruct frame */
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

        /* crc validation here */
        printf("validating: %s\n", argv[1]);

        uint32_t msgid = frame_msgid(&tlog.frame);
        uint8_t crc_extra; 

        // get crc_extra
        if(!mavlink_crc_extra_for(msgid, &crc_extra)) {
            continue;
        }
        
        // validate crc
        if (!mavlink_frame_crc_valid(&tlog.frame, crc_extra)) {
            fprintf(stderr, "invalid crc for message %u\n", msgid);
            continue;
        }

        putchar('\n');

        /* actual decoding happens here */
        switch(msgid) {
            case 30: {
                // attitude decode -> save
                MAVLinkAttitude_t attitude;
                mavlink_decode_attitude(&tlog.frame, &attitude);

                flight_data_add_attitude(&flight_data, tlog.timestamp_us, &attitude);
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


bool flight_data_add_attitude(FlightData_t *flight_data, uint64_t timestamp_us, MAVLinkAttitude_t *attitude) {
    if (flight_data == NULL) {
        return false;
    }

    // if we need more memory
    if (flight_data->attitude_capacity == flight_data->attitude_count) {
        // default memory: 256 for now
        size_t new_capacity = (flight_data->attitude_capacity == 0) ? 256 : flight_data->attitude_capacity * 2;

        AttitudeSample_t *new_samples = realloc(flight_data->attitudes, new_capacity * sizeof(*new_samples));

        if (new_samples == NULL) {
            return false;
        }

        // if reallocation success, overwrite
        flight_data->attitudes = new_samples;
        flight_data->attitude_capacity = new_capacity;
    }

    // then we need to finally append the datase
    // sample points to the next "empty" index
    AttitudeSample_t *sample = &flight_data->attitudes[flight_data->attitude_count];
    // put datas in sample
    sample->timestamp_us = timestamp_us;
    // copy the entire attitude under sample
    sample->attitude = *attitude;
    flight_data->attitude_count++;

    return true;
}

void flight_data_free(FlightData_t *flight_data) {
    if (flight_data == NULL) {
        return;
    }

    free (flight_data->attitudes);
    flight_data->attitudes = NULL;
    flight_data->attitude_count = 0;
    flight_data->attitude_capacity = 0;
}
