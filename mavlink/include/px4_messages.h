#ifndef PX4_MESSAGES_H
#define PX4_MESSAGES_H

#include <arpa/inet.h>
#include <stdbool.h>

#include "mavlink_parser.h"
#include "mavlink_types.h"

typedef struct {
    bool heartbeat_received;
    MAVLinkHeartbeat_t heartbeat;
    bool command_ack_received;
    MAVLinkCommandAck_t command_ack;
} PX4ReceivedMessages_t;

bool receive_px4_messages(
    int fd,
    const struct sockaddr_in *px4_address,
    MAVLinkParser_t *parser,
    PX4ReceivedMessages_t *received_messages
);

#endif
