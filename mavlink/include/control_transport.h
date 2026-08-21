#ifndef CONTROL_TRANSPORT_H
#define CONTROL_TRANSPORT_H

#include <arpa/inet.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

typedef struct ControlTransport ControlTransport_t;

typedef enum {
    CONTROL_TRANSPORT_NONE,
    CONTROL_TRANSPORT_UDP,
    CONTROL_TRANSPORT_SERIAL
} ControlTransportKind_t;

typedef bool (*ControlTransportSendFn)(
    ControlTransport_t *transport,
    const uint8_t *bytes,
    size_t length
);

typedef ssize_t (*ControlTransportReceiveFn)(
    ControlTransport_t *transport,
    uint8_t *buffer,
    size_t capacity
);

typedef void (*ControlTransportAcceptPeerFn)(ControlTransport_t *transport);

struct ControlTransport {
    int fd;
    ControlTransportKind_t kind;
    ControlTransportSendFn send;
    ControlTransportReceiveFn receive;
    ControlTransportAcceptPeerFn accept_peer;

    union {
        struct {
            struct sockaddr_in peer;
            struct sockaddr_in last_sender;
            bool peer_known;
            bool last_sender_valid;
        } udp;

        struct {
            char device[128];
        } serial;
    } context;
};

bool control_transport_open_udp(ControlTransport_t *transport, uint16_t port);
bool control_transport_open_serial(ControlTransport_t *transport, const char *device);
bool control_transport_send(
    ControlTransport_t *transport,
    const uint8_t *bytes,
    size_t length
);
ssize_t control_transport_receive(
    ControlTransport_t *transport,
    uint8_t *buffer,
    size_t capacity
);
void control_transport_accept_peer(ControlTransport_t *transport);
void control_transport_describe_peer(
    const ControlTransport_t *transport,
    char *description,
    size_t capacity
);
int control_transport_fd(const ControlTransport_t *transport);
bool control_transport_close(ControlTransport_t *transport);

#endif
