#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>

#include "../../include/flight_data.h"
#include "../../include/tlog.h"
#include "../../include/validator.h"

/*
 * make -C ../../ && ../../flight-replay ../../out/log_20260817_15_06_49/telemetry.tlog
 */

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

    /* ---------- load flight data ---------- */
    FlightData_t flight_data = {0};
    TLogLoadStats_t stats = {0};

    printf("READING %s\n", argv[1]);

    if (!tlog_load_flight_data(file, &flight_data, &stats)) {
        fprintf(stderr, "failed to load tlog\n");
        exit_status = 1;
    }

    // print_read_summary(&flight_data, &stats);


    /* ---------- replay! ---------- */

    // for now, let's print local_position_ned

    // uint64_t initial_timestamp = 0;

    uint64_t initial_timestamp = flight_data.local_positions[0].timestamp_us;

    for (size_t i = 0; i < flight_data.local_position_count; i ++) {
        uint64_t elapsed = flight_data.local_positions[i].timestamp_us - initial_timestamp;
        double elapsed_seconds = elapsed / 1000000.0;

        // prepare the data
        double x = flight_data.local_positions[i].local_position.x;
        double y = flight_data.local_positions[i].local_position.y;
        double z = flight_data.local_positions[i].local_position.z;

        printf("%8fs x=%8f y=%8f z=%8f\n", elapsed_seconds, x, y, z);
    }


    /* ---------- close ---------- */
    flight_data_free(&flight_data);

    /* close the file */
    if (fclose(file) == EOF) {
        perror("fclose");
        return 1;
    }

    return exit_status;
}
