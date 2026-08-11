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

    FILE *attitude_file = NULL;
    if (!log_create(&attitude_file)) {
        close(fd);
        return 1;
    }

    /* initialize mavlink parser */
    MAVLinkParser_t parser;
    MAVLinkFrame_t completed_frame;

    mavlink_parser_init(&parser);

    uint8_t buffer[256];

    /* main loop */
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

            uint32_t msgid = frame_msgid(&completed_frame);
           
            /* testing CRC (attitude) */
            if (msgid != 30) {
                continue;
            }

            if(!mavlink_frame_crc_valid(&completed_frame, 39)) {
                fprintf(stderr, "Invalid Attitude CRC\n");
                continue; // skip loggin
            }


            MAVLinkAttitude_t attitude;

            if (mavlink_decode_attitude(&completed_frame, &attitude)) {
                log_write_attitude(attitude_file, &attitude);
            }
        }
    }

    fclose(attitude_file);
    close(fd);

    return 0;
}
