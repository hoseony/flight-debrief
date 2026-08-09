#ifndef LOG_H
#define LOG_H

#include <stdbool.h>
#include <stdio.h>
#include "mavlink_types.h"

bool log_create(FILE **file);
void log_write_attitude(FILE *file, const MAVLinkAttitude_t *attitude);

#endif
