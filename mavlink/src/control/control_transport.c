#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "../../include/control_transport.h"
#include "../../include/serial_port.h"
#include "../../include/udp_port.h"

static bool udp_send(
        ControlTransport_t *transport,
        const uint8_t *bytes,
        size_t length) {
    if (!transport->context.udp.peer_known) {
        return false;
    }

    ssize_t sent = sendto(
            transport->fd, bytes, length, 0,
            (const struct sockaddr *)&transport->context.udp.peer,
            sizeof(transport->context.udp.peer));

    return sent == (ssize_t)length;
}

static ssize_t udp_receive(
        ControlTransport_t *transport,
        uint8_t *buffer,
        size_t capacity) {
    while (true) {
        struct sockaddr_in sender = {0};
        socklen_t sender_length = sizeof(sender);

        ssize_t received = recvfrom(
                transport->fd, buffer, capacity, MSG_DONTWAIT,
                (struct sockaddr *)&sender, &sender_length);

        if (received < 0) {
            return -1;
        }

        transport->context.udp.last_sender = sender;
        transport->context.udp.last_sender_valid = true;

        if (transport->context.udp.peer_known
                && (sender.sin_addr.s_addr
                    != transport->context.udp.peer.sin_addr.s_addr
                    || sender.sin_port != transport->context.udp.peer.sin_port)) {
            continue;
        }

        return received;
    }
}

static void udp_accept_peer(ControlTransport_t *transport) {
    if (!transport->context.udp.last_sender_valid) {
        return;
    }

    transport->context.udp.peer = transport->context.udp.last_sender;
    transport->context.udp.peer_known = true;
}

static bool serial_send(
        ControlTransport_t *transport,
        const uint8_t *bytes,
        size_t length) {
    size_t offset = 0;

    while (offset < length) {
        ssize_t sent = write(transport->fd, bytes + offset, length - offset);

        if (sent > 0) {
            offset += (size_t)sent;
            continue;
        }

        if (sent < 0 && errno == EINTR) {
            continue;
        }

        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd event = {
                .fd = transport->fd,
                .events = POLLOUT
            };

            int result = poll(&event, 1, 20);
            if (result > 0 && (event.revents & POLLOUT)) {
                continue;
            }
        }

        return false;
    }

    return true;
}

static ssize_t serial_receive(
        ControlTransport_t *transport,
        uint8_t *buffer,
        size_t capacity) {
    return read(transport->fd, buffer, capacity);
}

static void serial_accept_peer(ControlTransport_t *transport) {
    (void)transport;
}

bool control_transport_open_udp(ControlTransport_t *transport, uint16_t port) {
    if (transport == NULL) {
        return false;
    }

    *transport = (ControlTransport_t){.fd = -1};

    int fd = udp_port_open(port);
    if (fd < 0) {
        return false;
    }

    transport->fd = fd;
    transport->kind = CONTROL_TRANSPORT_UDP;
    transport->send = udp_send;
    transport->receive = udp_receive;
    transport->accept_peer = udp_accept_peer;
    return true;
}

bool control_transport_open_serial(
        ControlTransport_t *transport,
        const char *device) {
    if (transport == NULL || device == NULL) {
        return false;
    }

    *transport = (ControlTransport_t){.fd = -1};

    int fd = serial_port_open(device, B115200);
    if (fd < 0) {
        return false;
    }

    int flags = fcntl(fd, F_GETFL);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(fd);
        return false;
    }

    transport->fd = fd;
    transport->kind = CONTROL_TRANSPORT_SERIAL;
    transport->send = serial_send;
    transport->receive = serial_receive;
    transport->accept_peer = serial_accept_peer;
    snprintf(transport->context.serial.device,
            sizeof(transport->context.serial.device), "%s", device);
    return true;
}

bool control_transport_send(
        ControlTransport_t *transport,
        const uint8_t *bytes,
        size_t length) {
    if (transport == NULL || transport->send == NULL
            || bytes == NULL || length == 0) {
        return false;
    }

    return transport->send(transport, bytes, length);
}

ssize_t control_transport_receive(
        ControlTransport_t *transport,
        uint8_t *buffer,
        size_t capacity) {
    if (transport == NULL || transport->receive == NULL
            || buffer == NULL || capacity == 0) {
        errno = EINVAL;
        return -1;
    }

    return transport->receive(transport, buffer, capacity);
}

void control_transport_accept_peer(ControlTransport_t *transport) {
    if (transport != NULL && transport->accept_peer != NULL) {
        transport->accept_peer(transport);
    }
}

void control_transport_describe_peer(
        const ControlTransport_t *transport,
        char *description,
        size_t capacity) {
    if (description == NULL || capacity == 0) {
        return;
    }

    if (transport == NULL) {
        snprintf(description, capacity, "unknown");
        return;
    }

    if (transport->kind == CONTROL_TRANSPORT_UDP) {
        char address[INET_ADDRSTRLEN] = "unknown";
        inet_ntop(AF_INET, &transport->context.udp.peer.sin_addr,
                address, sizeof(address));
        snprintf(description, capacity, "udp://%s:%u",
                address, ntohs(transport->context.udp.peer.sin_port));
        return;
    }

    snprintf(description, capacity, "serial:%s",
            transport->context.serial.device);
}

int control_transport_fd(const ControlTransport_t *transport) {
    return transport == NULL ? -1 : transport->fd;
}

bool control_transport_close(ControlTransport_t *transport) {
    if (transport == NULL || transport->fd < 0) {
        return true;
    }

    bool success = close(transport->fd) == 0;
    transport->fd = -1;
    transport->kind = CONTROL_TRANSPORT_NONE;
    return success;
}
