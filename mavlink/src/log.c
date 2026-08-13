#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>
#include <inttypes.h>

#include "../include/mavlink_types.h"
#include "../include/log.h"

/* setting up dirs and files for the logs */
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
        return NULL;
    }

    char path[256];

    int length = snprintf(path, sizeof(path), "%s/%s", logs->session_directory, filename);

    if (length < 0 || (size_t)length >= sizeof(path)) {
        fprintf(stderr, "log path is too long\n");
        return NULL;
    }

    FILE *file = fopen(path, mode);

    if (file == NULL) {
        perror(path);
    }

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
        perror("log_open failed... :(\n");
        return false;
    }

    /* write header */
    // I am NEVER doing this again :(((
    fprintf(logs->attitude, "time_boot_ms,roll_rad,pitch_rad,yaw_rad,rollspeed_rad,pitchspeed_s,yawspeed_rad_s\n");
    fprintf(logs->attitude_quaternion, "time_boot_ms,q1,q2,q3,q4,rollspeed_rad_s,pitchspeed_rad_s,yawspeed_rad_s\n");
    fprintf(logs->local_position, "time_boot_ms,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s\n");
    fprintf(logs->global_position, "time_boot_ms,lat_deg_e7,lon_deg_e7,alt_mm,relative_alt_mm,vx_cm_s,vy_cm_s,vz_cm_s,hdg_cdeg\n");
    fprintf(logs->position_target, "time_boot_ms,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,afx_m_s2,afy_m_s2,afz_m_s2,yaw_rad,yaw_rate_rad_s,type_mask,coordinate_frame\n");

    return true;
}

// we want to for example get
// pointer to the log->attitude which is also a pointer and close that
// made it static 
void close_file(FILE **file) {
    if (file != NULL && *file != NULL) {
        fclose(*file);
    }
}

void log_close(MAVLinkLogs_t *logs) {
    if (logs == NULL) {
        return;
    }

    close_file(&logs->telemetry);
    close_file(&logs->attitude);
    close_file(&logs->attitude_quaternion);
    close_file(&logs->local_position);
    close_file(&logs->global_position);
    close_file(&logs->position_target);
}

/* Example Code (or test function)
int main() {
    MAVLinkLogs_t logs;
    create_out_directory();
    log_open(&logs);

    printf("testing create out & session dir\n");
}
*/


/* writing to the actual log files */
// OK, wth am I even doing at this point
void log_write_attitude(FILE *file, const MAVLinkAttitude_t *attitude) {
    if (file == NULL || attitude == NULL) {
        return;
    }

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

void log_write_attitudeQuaternion(FILE *file, const MAVLinkAttitudeQuaternion_t *attitude) {
    if (file == NULL || attitude == NULL) {
        return;
    }

    fprintf(
        file,
        "%" PRIu32 ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        attitude->time_boot_ms,
        attitude->q1,
        attitude->q2,
        attitude->q3,
        attitude->q4,
        attitude->rollspeed,
        attitude->pitchspeed,
        attitude->yawspeed
    );

    fflush(file);
}

void log_write_localPositionNed(FILE *file, const MAVLinkLocalPositionNed_t *position) {
    if (file == NULL || position == NULL) {
        return;
    }

    fprintf(
        file,
        "%" PRIu32 ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        position->time_boot_ms,
        position->x,
        position->y,
        position->z,
        position->vx,
        position->vy,
        position->vz
    );

    fflush(file);
}

void log_write_globalPositionInt(FILE *file,const MAVLinkGlobalPositionInt_t *position) {
    if (file == NULL || position == NULL) {
        return;
    }

    fprintf(
        file,
        "%" PRIu32 ",%" PRId32 ",%" PRId32 ","
        "%" PRId32 ",%" PRId32 ","
        "%" PRId16 ",%" PRId16 ",%" PRId16 ",%" PRIu16 "\n",
        position->time_boot_ms,
        position->lat,
        position->lon,
        position->alt,
        position->relative_alt,
        position->vx,
        position->vy,
        position->vz,
        position->hdg
    );

    fflush(file);
}

void log_write_positionTargetLocalNed(FILE *file, const MAVLinkPositionTargetLocalNed_t *target) {
    if (file == NULL || target == NULL) {
        return;
    }

    fprintf(
        file,
        "%" PRIu32 ","
        "%.9g,%.9g,%.9g,"
        "%.9g,%.9g,%.9g,"
        "%.9g,%.9g,%.9g,"
        "%.9g,%.9g,"
        "%" PRIu16 ",%u\n",
        target->time_boot_ms,
        target->x,
        target->y,
        target->z,
        target->vx,
        target->vy,
        target->vz,
        target->afx,
        target->afy,
        target->afz,
        target->yaw,
        target->yaw_rate,
        target->type_mask,
        (unsigned int)target->coordinate_frame
    );

    fflush(file);
}
