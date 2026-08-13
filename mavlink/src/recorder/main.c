#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>

#include "../../include/serial_port.h"
#include "../../include/log.h"
#include "../../include/mavlink_parser.h"
#include "../../include/mavlink_decode.h"
// #include "../include/mavlink_dispatch.h"

/* sigint is a interrupt signal that user can send with pressing ctrl+c
 * I need to correctly handle this so everything is closed and etc...
 */
static volatile sig_atomic_t stop_requested = 0;

static void handle_sigint(int signal_number) {
    (void)signal_number;
    stop_requested = 1;
}

int main(void) {
    // signal vector "template" used in sigaction call
    // initialize the struct
    struct sigaction action = {0};
   
    // we give the function to the handler
    action.sa_handler = handle_sigint;
    sigemptyset(&action.sa_mask); // this initializes signalmask to empty
    action.sa_flags = 0;

    // sigaction allows the action to be associated with a specific signal
    // here the siganl is SIGINT, 
    // It is like saying to the kernel that
    // For this process, when SIGINT is fired, use the signal disposition described by "action"
    if (sigaction(SIGINT, &action, NULL) == -1) {
        perror("sigaction");
        return 1;
    }

    /* open serial port */
    int fd = serial_port_open("/dev/serial0", B115200);

    if (fd < 0) {
        perror("serial_port_open");
        return 1;
    }
    printf("Serial Port Opened: fd = %d\n", fd);

    /* create log files */
    MAVLinkLogs_t logs;
    
    if (!log_tlog_open(&logs)) {
        close(fd);
        return 1;
    }

    /* initialize mavlink parser */
    MAVLinkParser_t parser;
    MAVLinkFrame_t completed_frame;
    mavlink_parser_init(&parser);

    /* main loop */
    uint8_t buffer[256];
    bool running = true;

    while(running && !stop_requested) {
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

            if (!log_write_frame(&logs, &completed_frame)) {
                fprintf(stderr, "failed to write telemetry frame\n");
                running = false;
                break;
            }

            /*
            maybe I should wrap this part into another function
            uint32_t msgid = frame_msgid(&completed_frame);
            uint8_t crc_extra; 

            crc validatoin
            // get crc_extra
            if(!mavlink_crc_extra_for(msgid, &crc_extra)) {
                continue;
            }
            
            // validate crc
            if (!mavlink_frame_crc_valid(&completed_frame, crc_extra)) {
                fprintf(stderr, "invalid crc for message %u\n", msgid);
                continue;
            }
            */

            /* csv files */
            // mavlink_handle_frame(&completed_frame, &logs, msgid);
        }
    }

    log_tlog_close(&logs);
    close(fd);

    return 0;
}
