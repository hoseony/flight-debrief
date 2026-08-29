#ifndef MAVLINK_DISPATCH_H
#define MAVLINK_DISPATCH_H

#include "mavlink_decode.h"
#include "log.h"

void mavlink_handle_frame(const MAVLinkFrame_t *frame, const MAVLinkLogs_t *logs, uint32_t msgid);

#endif
