#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "../include/mavlink_types.h"

bool log_create(FILE **file) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    char time_str[64];
    strftime(time_str, sizeof(time_str), "../out/log_%Y%m%d_%H_%M_%S.csv", t);
   
    *file = fopen(time_str, "w");
    
    if (*file == NULL) {
        perror("fopen");
        return false;
    }

    return true;
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
