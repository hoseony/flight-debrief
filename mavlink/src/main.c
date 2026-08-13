#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include "../include/serial_port.h"
#include "../include/log.h"
#include "../include/mavlink_parser.h"
#include "../include/mavlink_decode.h"

int main(void) {
    /* open serial port */
    int fd = serial_port_open("/dev/serial0", B115200);

    if (fd < 0) {
        perror("serial_port_open");
        return 1;
    }
    printf("Serial Port Opened: fd = %d\n", fd);

    /* create log files */
    create_out_directory();
    MAVLinkLogs_t logs;
    log_open(&logs);

    /* initialize mavlink parser */
    MAVLinkParser_t parser;
    MAVLinkFrame_t completed_frame;
    mavlink_parser_init(&parser);

    /* main loop */
    uint8_t buffer[256];

    while(1){
        ssize_t bytes_read = read(fd, buffer, sizeof(buffer));

        // error handling
        if (bytes_read < 0) {
            /* errno is a value that C/POSIX functions use to explain why an operation failed
             * EINTR is when it was interrupted
             */
            if (errno == EINTR) {
            // EINTR: interrupted system call -> ket's retry
                continue;
            }

            perror("read");
            break;
        }

        // if read success
        for (ssize_t i = 0; i < bytes_read; i++) {
            bool frame_ready = mavlink_parser_consume(&parser, buffer[i], &completed_frame);

            if (!frame_ready) {
                continue;
            }

            /* maybe I should wrap this part into another function */
            uint32_t msgid = frame_msgid(&completed_frame);
            uint8_t crc_extra; 

            /* crc validatoin */
            // get crc_extra
            if(!mavlink_crc_extra_for(msgid, &crc_extra)) {
                continue;
            }
            
            // validate crc
            if (!mavlink_frame_crc_valid(&completed_frame, crc_extra)) {
                fprintf(stderr, "invalid crc for message %u\n", msgid);
                continue;
            }

            /* log it */
        }
    }

    log_close(&logs);
    close(fd);

    return 0;
}
