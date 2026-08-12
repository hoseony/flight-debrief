#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

#include "../include/mavlink_types.h"
#include "../include/log.h"

// making files and directories in c is weird
bool create_out_directory(void) {
    if (mkdir("out", 0755) == 0) {
        return true;
    }

    if (errno == EEXIST) {
        return true;
    }

    perror("mkdir out");
    return false;
}

bool create_session_directory(MAVLinkLogs_t *logs) {
    if (logs == NULL) {
        return false;
    }
    
    // ensure parent dir exist...
    if (mkdir("out", 0755) == -1 && errno != EEXIST) {
        perror("mkdir out");
        return false;
    }

    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    if (strftime(logs->session_directory, sizeof(logs->session_directory), "out/log_%Y%m%d_%H_%M_%S", t) == 0) {
        fprintf(stderr, "session directory path is too long");
        return false;
    }

    if (mkdir(logs->session_directory, 0755) == -1) {
        if (errno == EEXIST) {
            fprintf(stderr, "session directory already exists: %s\n", logs->session_directory);
        } else {
            perror("mkdir session directory");
        }

        return false;
    }

    return false;
}

void log_write_attitude(FILE *file, const MAVLinkAttitude_t *attitude) {
    fprintf(
        file,
        "%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        attitude->time_boot_ms,
        attitude->roll,
        attitude->pitch,
        attitude->yaw,
        attitude->rollspeed,
        attitude->pitchspeed,
        attitude->yawspeed
    );
    
    fflush(file);
}

/*
int main() {
    create_out_directory();
    MAVLinkLogs_t logs;
    create_session_directory(&logs);

    printf("testing create out & session dir");
}
*/
