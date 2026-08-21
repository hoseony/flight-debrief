CC := cc

MAVLINK_GENERATED_DIR ?= ../../third-party/PX4-Autopilot/build/px4_sitl_default/mavlink
RECORDER_TARGET := flight-debrief
VALIDATOR_TARGET := tlog-validator
REPLAY_TARGET := flight-replay
CONTROL_TARGET := sitl-position-hold
BUILD_DIR := build

CPPFLAGS := -Iinclude -D_DEFAULT_SOURCE
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -O2 -g

COMMON_SOURCES := \
	src/common/flight_data.c \
	src/common/log.c \
	src/common/mavlink_decode.c \
	src/common/mavlink_dispatch.c \
	src/common/mavlink_parser.c \
	src/common/tlog.c

RECORDER_SOURCES := \
	src/recorder/main.c \
	src/recorder/serial_port.c \
	src/recorder/udp_port.c

VALIDATOR_SOURCES := src/validator/main.c \
					 src/validator/validator.c

REPLAY_SOURCES := \
	src/replay/main.c \
	src/replay/replay.c

CONTROL_SOURCES := src/control/main.c \
				   src/control/control_replay.c \
				   src/control/control_transport.c \
				   src/control/mavlink_encoder.c \
				   src/control/px4_commands.c \
				   src/control/px4_messages.c

COMMON_OBJECTS := $(COMMON_SOURCES:%.c=$(BUILD_DIR)/%.o)
RECORDER_OBJECTS := $(RECORDER_SOURCES:%.c=$(BUILD_DIR)/%.o)
VALIDATOR_OBJECTS := $(VALIDATOR_SOURCES:%.c=$(BUILD_DIR)/%.o)
REPLAY_OBJECTS := $(REPLAY_SOURCES:%.c=$(BUILD_DIR)/%.o)
CONTROL_OBJECTS := $(CONTROL_SOURCES:%.c=$(BUILD_DIR)/%.o)
OBJECTS := $(COMMON_OBJECTS) $(RECORDER_OBJECTS) $(VALIDATOR_OBJECTS) $(REPLAY_OBJECTS) $(CONTROL_OBJECTS)
DEPS := $(OBJECTS:.o=.d)

.PHONY: all recorder validator replay control clean

all: recorder validator replay control

recorder: $(RECORDER_TARGET)

validator: $(VALIDATOR_TARGET)

replay: $(REPLAY_TARGET)

control: $(CONTROL_TARGET)

$(RECORDER_TARGET): $(COMMON_OBJECTS) $(RECORDER_OBJECTS)
	$(CC) $^ -o $@

$(VALIDATOR_TARGET): $(COMMON_OBJECTS) $(VALIDATOR_OBJECTS)
	$(CC) $^ -o $@

$(REPLAY_TARGET): $(COMMON_OBJECTS) $(REPLAY_OBJECTS)
	$(CC) $^ -o $@

$(CONTROL_TARGET): $(COMMON_OBJECTS) \
	$(BUILD_DIR)/src/replay/replay.o \
	$(BUILD_DIR)/src/recorder/serial_port.o \
	$(BUILD_DIR)/src/recorder/udp_port.o \
	$(CONTROL_OBJECTS)
	$(CC) $^ -o $@

$(BUILD_DIR)/src/control/mavlink_encoder.o: CPPFLAGS += -isystem $(MAVLINK_GENERATED_DIR)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@


clean:
	rm -rf $(BUILD_DIR) $(RECORDER_TARGET) $(VALIDATOR_TARGET) $(REPLAY_TARGET) $(CONTROL_TARGET)

-include $(DEPS)
