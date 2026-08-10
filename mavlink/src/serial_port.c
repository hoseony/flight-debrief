#include "../include/serial_port.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>

int serial_port_open(const char *device, speed_t baud_rate) {
    int fd = open(device, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    } /*else {
        printf("Serial Port Opened: fd = %d\n", fd);
    }*/

    struct termios options;

    if (tcgetattr(fd, &options) == -1) {
        perror("tcgetattr");
        close(fd);
        return 1;
    }

    cfmakeraw(&options);

    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    options.c_cflag |= CLOCAL;   // ignore modem-control signals
    options.c_cflag |= CREAD;    // enable receiver

    // setting the uart to 8N1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cflag &= ~PARENB;  // no parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit

    options.c_cflag &= ~CRTSCTS; // disable RTS/CTS

    /* these options determine the behavior of read() */
    options.c_cc[VMIN] = 1;  // minimum number of bytes read() wants
    options.c_cc[VTIME] = 0; // timeout (in tenths of a second)

    tcflush(fd, TCIFLUSH); /* flush input buffer */

    /* Apply the settings */ 
    if (tcsetattr(fd, TCSANOW, &options) == -1) {
        perror("tcsetattr");
        close(fd);
        return 1;
    }; 

    return fd;
}
