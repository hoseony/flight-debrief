#ifndef VALIDATOR_H
#define VALIDATOR_H

#include <stdint.h>
#include <stdio.h>

#include "tlog.h"
#include "flight_data.h"

bool validator_handle_frame(TLogRecord_t *tlog, FlightData_t *flight_data, uint32_t msgid);
void print_read_summary(FlightData_t *flight_data, size_t records_read, size_t valid_frames, size_t invalid_crc, size_t unknown_crc);

#endif
