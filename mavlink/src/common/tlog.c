#include <stdbool.h>
#include <stdio.h>

#include "../../include/tlog.h"

/// read tlog from file and construct write it on the tlog struct.
/// bascially, frame reader
bool validator_read_frame(FILE *file, TLogRecord_t *tlog) {
    if (fread(tlog->frame.bytes, 1, 3, file) != 3) {
        return false;
    }

    size_t frame_length = 10 + tlog->frame.bytes[1] + 2;
    if (tlog->frame.bytes[2] & 0x01) {
        frame_length += 13;
    }

    size_t remaining = frame_length -3;
    if (fread(&tlog->frame.bytes[3], 1, remaining, file) != remaining) {
        return false;;
    }

    tlog->frame.length = frame_length;

    return true;
}

TLogReadResult_t tlog_read(FILE *file, TLogRecord_t *tlog) {
    if (file == NULL || tlog == NULL) {
        return TLOG_READ_ERROR;
    }

    uint8_t timestamp_bytes[8];
    size_t count = fread(timestamp_bytes, 1, 8, file);

    if (count == 0 && feof(file)) {
        return TLOG_READ_EOF;
    }

    if (count != 8) {
        return TLOG_READ_ERROR;
    }
    
    tlog->timestamp_us = 0;

    for (size_t i = 0; i < 8; i++) {
        tlog->timestamp_us = (tlog->timestamp_us << 8) | timestamp_bytes[i];
    }

    if (!validator_read_frame(file, tlog)) {
        return TLOG_READ_ERROR;
    }

    return TLOG_READ_OK;
}
