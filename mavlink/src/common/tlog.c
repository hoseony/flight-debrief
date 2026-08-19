#include <stdbool.h>
#include <stdio.h>

#include "../../include/tlog.h"
#include "../../include/mavlink_decode.h"

/// read tlog from file and construct write it on the tlog struct.
/// bascially, frame reader
static bool tlog_read_frame(FILE *file, TLogRecord_t *tlog) {
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

    if (!tlog_read_frame(file, tlog)) {
        return TLOG_READ_ERROR;
    }

    return TLOG_READ_OK;
}

static bool tlog_decode_and_store(const TLogRecord_t *tlog, FlightData_t *flight_data, uint32_t msgid) {
    switch (msgid) {
        case 0: {
            MAVLinkHeartbeat_t heartbeat;

            if (!mavlink_decode_heartbeat(&tlog->frame, &heartbeat)) {
                return false;
            }

            return flight_data_add_heartbeat(flight_data, tlog->timestamp_us, &heartbeat);
        }

        case 30: {
            MAVLinkAttitude_t attitude;

            if (!mavlink_decode_attitude(&tlog->frame, &attitude)) {
                return false;
            }

            return flight_data_add_attitude(flight_data, tlog->timestamp_us, &attitude);
        }

        case 31: {
            MAVLinkAttitudeQuaternion_t attitude;

            if (!mavlink_decode_attitudeQuaternion(&tlog->frame, &attitude)) {
                return false;
            }

            return flight_data_add_attitude_quaternion(flight_data, tlog->timestamp_us, &attitude);
        }

        case 32: {
            MAVLinkLocalPositionNed_t position;

            if (!mavlink_decode_localPositionNed(&tlog->frame, &position)) {
                return false;
            }

            return flight_data_add_local_position(flight_data, tlog->timestamp_us, &position);
        }

        case 33: {
            MAVLinkGlobalPositionInt_t position;

            if (!mavlink_decode_globalPositionInt(&tlog->frame, &position)) {
                return false;
            }

            return flight_data_add_global_position(flight_data, tlog->timestamp_us, &position);
        }

        case 36: {
            MAVLinkServoOutputRaw_t servo_output;

            if (!mavlink_decode_servo_output_raw(&tlog->frame, &servo_output)) {
                return false;
            }

            return flight_data_add_servo_output(flight_data, tlog->timestamp_us, &servo_output);
        }

        case 83: {
            MAVLinkAttitudeTarget_t attitude_target;

            if (!mavlink_decode_attitude_target(&tlog->frame, &attitude_target)) {
                return false;
            }

            return flight_data_add_attitude_target(flight_data, tlog->timestamp_us, &attitude_target);
        }

        case 84: {
            MAVLinkSetPositionTargetLocalNed_t target;

            if (!mavlink_decode_setPositionTargetLocalNed(&tlog->frame, &target)) {
                return false;
            }

            return flight_data_add_set_position_target(flight_data, tlog->timestamp_us, &target);
        }

        case 85: {
            MAVLinkPositionTargetLocalNed_t target;

            if (!mavlink_decode_positionTargetLocalNed(&tlog->frame, &target)) {
                return false;
            }

            return flight_data_add_position_target(flight_data, tlog->timestamp_us, &target);
        }

        default:
            return true;
    }
}

static void tlog_count_unknown_message(TLogLoadStats_t *stats, uint32_t msgid) {
    for (size_t i = 0; i < stats->unknown_message_count; i++) {
        if (stats->unknown_messages[i].msgid == msgid) {
            stats->unknown_messages[i].count++;
            return;
        }
    }

    if (stats->unknown_message_count < TLOG_MAX_UNKNOWN_MESSAGE_TYPES) {
        TLogMessageCount_t *message = &stats->unknown_messages[stats->unknown_message_count];
        message->msgid = msgid;
        message->count = 1;
        stats->unknown_message_count++;
    }
}

bool tlog_load_flight_data(FILE *file, FlightData_t *flight_data, TLogLoadStats_t *stats) {
    if (file == NULL || flight_data == NULL || stats == NULL) {
        return false;
    }

    *flight_data = (FlightData_t){0};
    *stats = (TLogLoadStats_t){0};

    while (true) {
        TLogRecord_t tlog = {0};
        TLogReadResult_t read_result = tlog_read(file, &tlog);

        if (read_result == TLOG_READ_EOF) {
            break;
        }

        if (read_result == TLOG_READ_ERROR) {
            return false;
        }

        stats->records_read++;

        uint32_t msgid = frame_msgid(&tlog.frame);
        uint8_t crc_extra;

        if (!mavlink_crc_extra_for(msgid, &crc_extra)) {
            stats->unknown_crc++;
            tlog_count_unknown_message(stats, msgid);
            continue;
        }

        if (!mavlink_frame_crc_valid(&tlog.frame, crc_extra)) {
            stats->invalid_crc++;
            continue;
        }

        stats->valid_frames++;

        if (!tlog_decode_and_store(&tlog, flight_data, msgid)) {
            stats->decode_failures++;
        }
    }

    return true;
}
