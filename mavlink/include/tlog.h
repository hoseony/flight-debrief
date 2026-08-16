#ifndef TLOG_H
#define TLOG_H

#include <stdint.h>
#include "mavlink_types.h"

/// this struct will be used to completely decode the .tlog file 
/// .tlog file have 8bit timestamp followed by the frame (actual telemetry data).
typedef struct {
    uint64_t timestamp_us;
    MAVLinkFrame_t frame;
} TLogRecord_t;

/// This is used to capture errors for the tlog reader
/// enum: TLOG_READ_OK, TLOG_READ_EOF, TLOG_READ_ERROR
typedef enum {
    TLOG_READ_OK,
    TLOG_READ_EOF,
    TLOG_READ_ERROR
} TLogReadResult_t;


#endif
