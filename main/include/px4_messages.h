#ifndef PX4_MESSAGES_H
#define PX4_MESSAGES_H

#include <stdbool.h>

#include "control_transport.h"
#include "mavlink_parser.h"
#include "mavlink_types.h"

typedef struct {
    bool heartbeat_received;
    MAVLinkHeartbeat_t heartbeat;
    bool local_position_received;
    MAVLinkLocalPositionNed_t local_position;
    bool command_ack_received;
    MAVLinkCommandAck_t command_ack;
} PX4ReceivedMessages_t;

bool receive_px4_messages(
    ControlTransport_t *transport,
    MAVLinkParser_t *parser,
    PX4ReceivedMessages_t *received_messages
);

#endif
