CC := cc

TARGET := flight-debrief
BUILD_DIR := build

CPPFLAGS := -Iinclude -D_DEFAULT_SOURCE
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -O2 -g

SOURCES := \
	src/main.c \
	src/serial_port.c \
	src/mavlink_parser.c \
	src/mavlink_decode.c \
	src/log.c

OBJECTS := $(SOURCES:src/%.c=$(BUILD_DIR)/%.o)
DEPS := $(OBJECTS:.o=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@

$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

-include $(DEPS)
