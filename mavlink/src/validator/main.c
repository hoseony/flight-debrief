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

    FlightData_t flight_data = {0};
    TLogLoadStats_t stats = {0};

    printf("READING %s\n", argv[1]);

    if (!tlog_load_flight_data(file, &flight_data, &stats)) {
        fprintf(stderr, "failed to load tlog\n");
        exit_status = 1;
    }

    print_read_summary(&flight_data, &stats);

    flight_data_free(&flight_data);

    /* close the file */
    if (fclose(file) == EOF) {
        perror("fclose");
        return 1;
    }

    return exit_status;
}
