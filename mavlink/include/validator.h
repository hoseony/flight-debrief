#ifndef VALIDATOR_H
#define VALIDATOR_H

#include <stdint.h>

#include "tlog.h"
#include "flight_data.h"

bool validator_handle_frame(TLogRecord_t *tlog, FlightData_t *flight_data, uint32_t msgid);

#endif
