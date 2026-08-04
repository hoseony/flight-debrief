/* Since this is the first time writing anything for Linux machines
 * This will contain too many comments than it needs to have.        */

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
 * 
 */

int main() {
    int fd, len;
    char text[255];
    struct termios options; /* serial ports setting */

    /* file descript = first serial port on linux 
     *                 flags: read and writ
     *                 non-blocking / returns immediatly if no bytes are available
     *                 no control tty */
    fd = open("/dev/serial0", O_RDWR | O_NDELAY | O_NOCTTY);

    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    /* Read current serial port settings */
    // tcgetattr(fd, &options);
   
    /* Set up serial port */
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR; /* ignore parity error */
    options.c_oflag = 0;
    options.c_lflag = 0;

    /* Apply the settings */ 
    tcflush(fd, TCIFLUSH); /* flush input buffer */
    tcsetattr(fd, TCSANOW, &options);

    /* Write from serial port */
    strcpy(text, "Hello from my RPi\n\r");
    len = strlen(text);
    len = write(fd, text, len);
    printf("wrote %d bytes over UART\n", len);

    printf("You have 5s to send me some input data... \n");
    sleep(5);

    /* Read from serial port */
    memset(text, 0, 255);
    len = read(fd, text, 255);
    printf("Received %d bytes\n", len);
    printf("Received string: %s\n", text);

    close(fd);
    return 0;
}
