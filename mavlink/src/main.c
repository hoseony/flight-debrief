#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#include "../include/types.h"

/*----- Function Prototypes -----*/
void crc_accumulate(unsigned char byte, uint16_t *crc);
uint32_t read_u32_le(uint8_t *bytes);
float read_f32_le(uint8_t *bytes);

/*----- Entry Point -----*/
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


    /* Open the file */
    FILE* attitude_log;
    attitude_log = fopen("../out/attitude.csv", "w");

    if (attitude_log == NULL) {
        perror("fopen attitude.csv");
        close(fd);
        return 1;
    }

    fprintf(attitude_log, "time,roll,pitch,yaw\n");
    fflush(attitude_log);

    /* Actual Buffer to read */
    unsigned char buffer[256];

    MAVLinkPacket_t packet;
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

                /* [At position == 3]
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

                    uint32_t message_id = (frame[7]) | (frame[8] << 8) | (frame[9] << 16);
                    /*
                    printf("Complete frame: %zu bytes\n", expected_len);

                    for (size_t i = 0; i < expected_len; i++) {
                        printf("%02X ", frame[i]);
                    }

                    putchar('\n');

                    reconstruct the message ID


                    // HEARTBEAT
                    // something is wrong if its hearbeat and len != 9
                    if (message_id == 0 && frame[1] == 9) {
                        MAVLinkHeartbeat_t heartbeat;
                        heartbeat.custom_mode = 
                            frame[10] | (frame[11] << 8) | (frame[12] << 16) | (frame[13] << 24);

                        heartbeat.type            = frame[14];
                        heartbeat.autopilot       = frame[15];
                        heartbeat.base_mode       = frame[16];
                        heartbeat.system_status   = frame[17];
                        heartbeat.mavlink_version = frame[18];

                        printf("== HEARTBEAT ==\n");
                        printf("  sysid:           %u\n",     frame[5]);
                        printf("  compid:          %u\n",     frame[6]);
                        printf("  sequence:        %u\n",     frame[4]);
                        printf("  custom_mode:     0x%08X\n", (unsigned int)heartbeat.custom_mode);
                        printf("  type:            %u\n",     heartbeat.type);
                        printf("  autopilot:       %u\n",     heartbeat.autopilot);
                        printf("  base_mode:       0x%02X\n", heartbeat.base_mode);
                        printf("  system_status:   %u\n",     heartbeat.system_status);
                        printf("  mavlink_version: %u\n\n",   heartbeat.mavlink_version);

                        // saving this into some file... how? idk
                    }
                    */

                    if (message_id == 30 && frame[1] == 28) {
                        MAVLinkAttitude attitude;

                        attitude.time_boot_ms = read_f32_le(&frame[10]);
                        attitude.roll         = read_f32_le(&frame[14]);
                        attitude.pitch        = read_f32_le(&frame[18]);
                        attitude.yaw          = read_f32_le(&frame[22]);
                        attitude.pitchspeed   = read_f32_le(&frame[26]);
                        attitude.rollspeed    = read_f32_le(&frame[30]);
                        attitude.yawspeed     = read_f32_le(&frame[34]);

                        const float rad_to_deg = 57.2957795f;
/*
                        printf("ATTITUDE\n");
                        printf("  time_boot_ms: %u ms\n",       (unsigned int)attitude.time_boot_ms);
                        printf("  roll:         %8.3f deg\n",   attitude.roll * rad_to_deg);
                        printf("  pitch:        %8.3f deg\n",   attitude.pitch * rad_to_deg);
                        printf("  yaw:          %8.3f deg\n",   attitude.yaw * rad_to_deg);
                        printf("  rollspeed:    %8.3f deg/s\n", attitude.rollspeed * rad_to_deg);
                        printf("  pitchspeed:   %8.3f deg/s\n", attitude.pitchspeed * rad_to_deg);
                        printf("  yawspeed:     %8.3f deg/s\n", attitude.yawspeed * rad_to_deg);
*/
                        /* sending the data to gnuplot */

                        double time_s    = attitude.time_boot_ms / 1000.0;
                        double roll_deg  = attitude.roll * rad_to_deg;
                        double pitch_deg = attitude.pitch * rad_to_deg;
                        double yaw_deg   = attitude.yaw * rad_to_deg;

                        fprintf(attitude_log, "%.3f, %.3f, %.3f, %.3f\n", time_s, roll_deg, pitch_deg, yaw_deg);

                        fflush(attitude_log);
                    }

                    // reset the state
                    state = WAIT_MAGIC;
                    position = 0;
                    expected_len = 0;
                }
            }
        }

        fflush(stdout);
    }

    fclose(attitude_log);
    close(fd);
    return 0;
}

/*----- Helper Functions -----*/

/// read 4 bytes consecutively and returns into one uint32_t
uint32_t read_u32_le(uint8_t *bytes) {
    return ((uint32_t)bytes[0] | (uint32_t)(bytes[1] << 8) 
            | (uint32_t)(bytes[2] << 16) | (uint32_t)(bytes[3] << 24));
}

float read_f32_le(uint8_t *bytes) {
    uint32_t bits = read_u32_le(bytes);
    float value;

    // memcpy!!
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/// CRC algorithm
void crc_accumulate(unsigned char byte, uint16_t *crc) {
    unsigned char tmp;

    tmp = byte ^ (unsigned char)(*crc & 0xFF);
    tmp ^= (unsigned char)(tmp << 4);

    *crc = (*crc >> 8)
         ^ ((uint16_t)tmp << 8)
         ^ ((uint16_t)tmp << 3)
         ^ ((uint16_t)tmp >> 4);
}
