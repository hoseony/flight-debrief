#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <arpa/inet.h>
#include <sys/socket.h>

// referenced example provided by mavlink!
// https://github.com/mavlink/mavlink/blob/master/examples/c/udp_example.c
int udp_port_open(uint16_t port) {
    // open UDP Socket
    int fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    // Bind to port 
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr));
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -1;
    }

    return fd;
}
