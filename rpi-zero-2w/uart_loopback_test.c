/* Since this is the first time writing anything for Linux machines
 * This will contain too many comments than it needs to have. 
 *
 * <Some Notes>
 * pins connected: pin8 - pin10
 * ssuze_t: signed integer, count of bytes or negative error status
 *
 * <References>
 * - https://github.com/Johannes4Linux/Linux_Embedded_Interfaces
 * - https://en.wikibooks.org/wiki/Serial_Programming/termios
 *
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

/* <terminos.h>
 * https://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html
 *  - contains definitions used by the terminal I/O interfaces
 *  struct terminos
 *      tcflag_t c_iflag    input   modes
 *      tcflag_t c_oflag    output  modes
 *      tcflag_t c_cflag    control modes
 *      tcflag_t c_lflag    local   modes
 *      cc_t     c_cc[NCCS] control chars
 */

int main() {
    struct termios options; /* serial ports setting */

    /* file descript = first serial port on linux 
     *                 flags: read and write
     *                 no control tty */
    int fd = open("/dev/serial0", O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    } else {
        printf("Connection to Port Opend fd = %d \n", fd);
    }

    /* Linux Manual Page of tcgetattr()
     * https://www.man7.org/linux/man-pages//man3/tcgetattr.3p.html
     *  - Applications should always do a tcgetattr(), save the termios
     *    structure values returned, and then do a tcsetattr(), changing
     *    only the necessary fields.
     */

    /* Read current serial port settings */
    // you should do this to initialize "options"
    if (tcgetattr(fd, &options) == -1) {
        perror("tcgetattr");
        close(fd);
        return 1;
    }
   
    /* Set up serial port (modify the config locally) */

    // 115200 bps, 8 data pits, ignore modem-control signals, enable receiver
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR; /* ignore parity error */
    options.c_oflag = 0;
    options.c_lflag = 0;

    /* these options determine the behavior of read() */
    options.c_cc[VMIN] = 0;   // minimum number of bytes read() wants
    options.c_cc[VTIME] = 20; // timeout (in tenths of a second)


    tcflush(fd, TCIFLUSH); /* flush input buffer */

    /* Apply the settings */ 
    if (tcsetattr(fd, TCSANOW, &options) == -1) {
        perror("tcsetattr");
        close(fd);
        return 1;
    };

    
    /* ============================================
     * we wish to check if 
     * Message travels throough TX -> Jumper -> RX 
     * ============================================ */

    char text[255];
    strcpy(text, "Hello from my RPi\r\n");
    size_t text_len = strlen(text);

    ssize_t written = write(fd, text, text_len);
    printf("Wrote %zd bytes over UART\n", written);

    if (tcdrain(fd) == -1) {
        perror("tcdrain");
        close(fd);
        return 1;
    }

    char received[255];

    size_t expected = text_len;
    size_t total_received = 0;

    while (total_received < expected) {
        // read consumes
        ssize_t received_len = read(fd, &received[total_received], expected - total_received); 

        if (received_len < 0) {
            perror("read");
            close(fd);
            return 1;
        }

        if (received_len == 0) {
            printf("FAIL: time out after receiving %zu / %zu bytes", total_received, expected);
            close(fd);
            return 1;
        }

        total_received += (size_t)received_len;
    }

    received[total_received] = '\0';
    printf("Received text: %.*s\n", (int)total_received, received);
   
    if ((size_t)total_received == text_len && memcmp(received, text, text_len) == 0) {
        // note that memcmp returns 0 if no difference!
        printf("PASS: uart loopback succeeded\n");
    } else {
        printf("FAIL: received data does not match transmitted data\n");
    }

    close(fd);
    return 0;
}
