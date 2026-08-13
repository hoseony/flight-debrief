#ifndef LOG_H
#define LOG_H

#include <stdbool.h>
#include <stdio.h>
#include "mavlink_types.h"

/// type to store logs in a organized way
typedef struct {
    char session_directory[100];
    FILE *telemetry;

    FILE *attitude;
    FILE *attitude_quaternion;
    FILE *local_position;
    FILE *global_position;
    FILE *position_target;
} MAVLinkLogs_t;

bool create_out_directory(void);
bool create_session_directory(MAVLinkLogs_t *logs);
FILE* create_log_file(const MAVLinkLogs_t *logs, const char *filename, const char *mode);

bool log_open(MAVLinkLogs_t *logs);
void log_close(MAVLinkLogs_t *logs);

bool log_write_frame(MAVLinkLogs_t *logs, const MAVLinkFrame_t *frame);

void log_write_attitude(FILE *file, const MAVLinkAttitude_t *attitude);
void log_write_attitudeQuaternion(FILE *file, const MAVLinkAttitudeQuaternion_t *attitude);
void log_write_localPositionNed(FILE *file, const MAVLinkLocalPositionNed_t *position);
void log_write_globalPositionInt(FILE *file, const MAVLinkGlobalPositionInt_t *position);
void log_write_positionTargetLocalNed(FILE *file, const MAVLinkPositionTargetLocalNed_t *target);
#endif
