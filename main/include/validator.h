#ifndef VALIDATOR_H
#define VALIDATOR_H

#include <stdint.h>
#include <stdio.h>

#include "tlog.h"
#include "flight_data.h"

void print_read_summary(const FlightData_t *flight_data, const TLogLoadStats_t *stats);

#endif
