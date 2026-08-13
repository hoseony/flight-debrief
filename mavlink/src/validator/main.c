#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>

int main(int argc, char* argv[]) {
    if (argc !=2) {
        fprintf(stderr, "usage: %s <telemetry.tlog>\n", argv[0]);
    }

    FILE *file;
    file = fopen(argv[1], "r");

    if (file == NULL) {
        fprintf(stderr, "cannot open %s: %s\n", argv[1], strerror(errno));
        return 1;
    }

    printf("validating: %s\n", argv[1]);

    /* crc validation here */
    

    // printing entire file
    uint8_t buffer[16];
    size_t offset = 0;
    size_t bytes_read;

    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        printf("%08zx  ", offset);

        for (size_t i = 0; i < bytes_read; i++) {
            printf("%02X ", buffer[i]);
        }

        putchar('\n');
        offset += bytes_read;
    }






    if (fclose(file) == EOF) {
        perror("fclose");
        return 1;
    }

    return 0;
}
