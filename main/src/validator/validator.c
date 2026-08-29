#include <stdio.h>

#include "../../include/validator.h"

void print_read_summary(const FlightData_t *flight_data, const TLogLoadStats_t *stats) {
    if (flight_data == NULL || stats == NULL) {
        return;
    }

    size_t decoded_total =
        flight_data->heartbeat_count +
        flight_data->attitude_count +
        flight_data->attitude_quaternion_count +
        flight_data->local_position_count +
        flight_data->global_position_count +
        flight_data->servo_output_count +
        flight_data->attitude_target_count +
        flight_data->set_position_target_count +
        flight_data->position_target_count;

    printf("\n");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10s |\n", "Validation summary", "Count");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10zu |\n", "Records read", stats->records_read);
    printf("| %-28s | %10zu |\n", "Valid frames", stats->valid_frames);
    printf("| %-28s | %10zu |\n", "Invalid CRC", stats->invalid_crc);
    printf("| %-28s | %10zu |\n", "Unknown CRC", stats->unknown_crc);
    printf("| %-28s | %10zu |\n", "Decode failures", stats->decode_failures);
    printf("+------------------------------+------------+\n");

    printf("\n");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10s |\n", "Decoded messages", "Count");
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10zu |\n", "Heartbeat (0)", flight_data->heartbeat_count);
    printf("| %-28s | %10zu |\n", "Attitude (30)", flight_data->attitude_count);
    printf("| %-28s | %10zu |\n", "Attitude quaternion (31)", flight_data->attitude_quaternion_count);
    printf("| %-28s | %10zu |\n", "Local position NED (32)", flight_data->local_position_count);
    printf("| %-28s | %10zu |\n", "Global position INT (33)", flight_data->global_position_count);
    printf("| %-28s | %10zu |\n", "Servo output raw (36)", flight_data->servo_output_count);
    printf("| %-28s | %10zu |\n", "Attitude target (83)", flight_data->attitude_target_count);
    printf("| %-28s | %10zu |\n", "Set position target (84)", flight_data->set_position_target_count);
    printf("| %-28s | %10zu |\n", "Position target (85)", flight_data->position_target_count);
    printf("+------------------------------+------------+\n");
    printf("| %-28s | %10zu |\n", "Total", decoded_total);
    printf("+------------------------------+------------+\n");

    if (stats->unknown_message_count > 0) {
        printf("\n--- Unknown message IDs ---\n");

        for (size_t i = 0; i < stats->unknown_message_count; i++) {
            printf("msgid %-6u : %zu\n", stats->unknown_messages[i].msgid, stats->unknown_messages[i].count);
        }
    }
}
