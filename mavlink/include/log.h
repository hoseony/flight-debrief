#ifndef LOG_H
#define LOG_H

#include <stdbool.h>
#include <stdio.h>
#include "mavlink_types.h"

/// type to store logs in a organized way
typedef struct {
    char session_directory[100];

    FILE *attitude;
    FILE *attitude_quaternion;
    FILE *local_position;
    FILE *global_position;
    FILE *position_target;
} MAVLinkLogs_t;

bool log_open(MAVLinkLogs_t *logs);
void log_write_attitude(FILE *file, const MAVLinkAttitude_t *attitude);

#endif
