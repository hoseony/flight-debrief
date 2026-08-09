#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>

int serial_port_open(const char *device, speed_t baud_rate);

#endif
