#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    int fd, len;
    struct termios options; /* serial ports setting */

    /* open a serial port */
    fd = open("/dev/serial0", O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    } else {
        printf("Serial Port Opend: fd = %d \n", fd);
    }

    /* Read current serial port settings */
    if (tcgetattr(fd, &options) == -1) { 
    // initializing local variable "options"
        perror("tcgetattr");
        close(fd);
        return 1;
    }
   
    /* Set up serial port */
    cfmakeraw(&options); // make it a raw
   
    // set baud rate: 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

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

    /* Actual Buffer to read */
    unsigned char buffer[256];

    while (1) {
        ssize_t count = read(fd, buffer, sizeof(buffer));

        if (count < 0) {
            perror("read error");
            break;
        }

        printf("%zd bytes:", count);

        for (ssize_t i = 0; i < count; i++) {
            printf(" %02X", buffer[i]);
        }

        putchar('\n');
        fflush(stdout);
    }

    close(fd);
    return 0;
}
