#include <string.h>
#include <stdbool.h>

#include "../../include/mavlink_types.h"

static void mavlink_parser_reset(MAVLinkParser_t *parser) {
    parser->state = WAIT_MAGIC;
    parser->position = 0;
    parser->expected_length = 0;
}

void mavlink_parser_init(MAVLinkParser_t *parser) {
    mavlink_parser_reset(parser);
}

/// This returns the "FRAME" of the MAVLink message.
/// It identifies the start and the end, and returns that through
/// MAVLinkFrame_t *completedframe
bool mavlink_parser_consume(MAVLinkParser_t *parser, uint8_t byte, MAVLinkFrame_t *completed_frame) {
    if (parser->state == WAIT_MAGIC) {
        if (byte == MAVLINK2_MAGIC) {
            parser->bytes[0] = byte;
            parser->position = 1;
            parser->expected_length = 0;
            parser->state = READ_FRAME;
        }

        return false;
    }

    if (parser->position >= sizeof(parser->bytes)) {
        mavlink_parser_reset(parser);
        return false;
    }
    
    parser->bytes[parser->position++] = byte;

    /* [At position == 3]
     * frame[0] = magic (0XFD)
     * frame[1] = *payload*_length
     * frame[2] = incompat_flags
     *          if 0X01 -> Signed! (13 more bytes)
     */

    if (parser->position == 3) {
        parser->expected_length = 10u + parser->bytes[1] + 2u;

        if (parser->bytes[2] & 0x01) {
            parser->expected_length += 13;
        }
    }

    if ((parser->expected_length != 0) && (parser->position == parser->expected_length)) {
        completed_frame->length = parser->expected_length;

        memcpy(completed_frame->bytes, parser->bytes, completed_frame->length);

        mavlink_parser_reset(parser);
        return true;
    }

    return false;
}
