#ifndef TLOG_H
#define TLOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "mavlink_types.h"
#include "flight_data.h"

#define TLOG_MAX_UNKNOWN_MESSAGE_TYPES 64

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

typedef struct {
    uint32_t msgid;
    size_t count;
} TLogMessageCount_t;

typedef struct {
    size_t records_read;
    size_t valid_frames;
    size_t invalid_crc;
    size_t unknown_crc;
    size_t decode_failures;
    TLogMessageCount_t unknown_messages[TLOG_MAX_UNKNOWN_MESSAGE_TYPES];
    size_t unknown_message_count;
} TLogLoadStats_t;

TLogReadResult_t tlog_read(FILE *file, TLogRecord_t *tlog);
bool tlog_load_flight_data(FILE *file, FlightData_t *flight_data, TLogLoadStats_t *stats);

#endif
