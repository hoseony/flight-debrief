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

    return true;
}

FILE* create_log_file(const MAVLinkLogs_t *logs, const char *filename, const char *mode) {
    if (logs == NULL || filename == NULL || mode == NULL) {
        return false;
    }

    char path[256];

    int length = snprintf(path, sizeof(path), "%s/%s", logs->session_directory, filename);

    if (length < 0 || (size_t)length >= sizeof(path)) {
        fprintf(stderr, "log path is too long\n");
        return false;
    }

    FILE *file = fopen(path, mode);

    if (file == NULL) {
        perror(path);
    }

/*
    logs->telemetry           = fopen("telemetry", "w");
    logs->attitude            = fopen("attitude", "w");
    logs->attitude_quaternion = fopen("attitude_quaternion", "w");
    logs->local_position      = fopen("local_position", "w");
    logs->global_position     = fopen("global_position", "w");
    logs->position_target     = fopen("position_target", "w");
*/
    return file;
}

bool log_open(MAVLinkLogs_t *logs) {
    if (logs == NULL) {
        return false;
    }

    *logs = (MAVLinkLogs_t){0};

    if (!create_session_directory(logs)) {
        return false;
    }

    logs->telemetry           = create_log_file(logs, "telemetry.tlog",      "wb");
    logs->attitude            = create_log_file(logs, "attitude",            "w");
    logs->attitude_quaternion = create_log_file(logs, "attitude_quaternion", "w");
    logs->local_position      = create_log_file(logs, "local_position",      "w");
    logs->global_position     = create_log_file(logs, "global_position",     "w");
    logs->position_target     = create_log_file(logs, "position_target",     "w");

    if (logs->telemetry == NULL || logs->attitude == NULL || logs->attitude_quaternion == NULL
            || logs->local_position == NULL || logs->global_position == NULL || logs->position_target == NULL) {
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

/* Example Code (or test function)
int main() {
    MAVLinkLogs_t logs;
    create_out_directory();
    log_open(&logs);

    printf("testing create out & session dir\n");
}
*/
