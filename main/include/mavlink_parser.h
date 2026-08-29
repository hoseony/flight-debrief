#ifndef MAVLINK_PARSER_H
#define MAVLINK_PARSER_H

#include "mavlink_types.h"
#include <stdbool.h>

void mavlink_parser_init(MAVLinkParser_t *parser);
bool mavlink_parser_consume(MAVLinkParser_t *parser, uint8_t byte, MAVLinkFrame_t *completed_frame);

#endif
