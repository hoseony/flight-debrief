#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

typedef enum State {
    WAIT_MAGIC, 
    READ_FRAME,
} state_t;


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
    cfmakeraw(&options); // raw: os does not touch the data
   
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

    state_t state = WAIT_MAGIC;
    unsigned char frame[280];
    size_t position = 0;
    size_t expected_len = 0;

    while (1) {
        ssize_t count = read(fd, buffer, sizeof(buffer));

        if (count < 0) {
            perror("read error");
            break;
        }

        if (count > 0) {
            // loop through the buffer
            for (size_t i = 0; i < count; i++) {
                unsigned char byte = buffer[i];

                // once we see 0XFD on wait stage,
                // move to the next stage, start collecting
                if (state == WAIT_MAGIC) {
                    if (byte == 0xFD) {
                        frame[0] = byte;
                        position = 1;
                        expected_len = 0;
                        state = READ_FRAME;
                    }

                    continue;
                }

                frame[position++] = byte;

                /* At position == 3
                 * frame[0] = magic (0XFD)
                 * frame[1] = *payload*_length
                 * frame[2] = incompat_flags
                 *          if 0X01 -> Signed! (13 more bytes)
                 */

                if (position == 3) {
                    // MAVLink packet size: 
                    // 10 (header) + payload + 2 check sum
                    expected_len = 10 + frame[1] + 2;
                    if (frame[2] & 0X01) {
                        expected_len += 13;
                    }
                }

                if ((expected_len != 0) && (position == expected_len)) {
                    printf("Complete frame: %zu bytes |", expected_len);

                    for (size_t i = 0; i < expected_len; i++) {
                        printf("%02X ", frame[i]);
                    }

                    putchar('\n');

                    // reset the state
                    state = WAIT_MAGIC;
                    position = 0;
                    expected_len = 0;
                }
            }
        }

        fflush(stdout);
    }

    close(fd);
    return 0;
}
