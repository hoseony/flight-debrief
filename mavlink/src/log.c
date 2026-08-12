#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

#include "../include/mavlink_types.h"
#include "../include/log.h"

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

bool create_session_directory(char *path, size_t path_size) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "./out/log_%Y%m%d_%H_%M_%S.csv", t);

    int result = sprintf(path, path_size, "out/%s", timestamp);
}
/*
bool log_create(FILE **file) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    char time_str[64];
    strftime(time_str, sizeof(time_str), "./out/log_%Y%m%d_%H_%M_%S.csv", t);
   
    *file = fopen(time_str, "w");
    
    if (*file == NULL) {
        perror("fopen");
        return false;
    }

    return true;
}
*/
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
