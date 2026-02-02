TARGET = fly_sitl
FC_ROOT = ../../..
SRCS = main.c module_sim.c \
       $(FC_ROOT)/base/foundation/platform.c \
       $(FC_ROOT)/base/foundation/pubsub.c \
       $(FC_ROOT)/modules/rc_receiver/rc_receiver.c \
       $(FC_ROOT)/modules/attitude_estimation/attitude_estimation.c \
       $(FC_ROOT)/modules/attitude_control/attitude_control.c \
       $(FC_ROOT)/modules/local_storage/local_storage.c \
       $(FC_ROOT)/modules/logger/logger.c \
       $(FC_ROOT)/modules/fault_handler/fault_handler.c \
       $(FC_ROOT)/modules/oscillation_detection/oscillation_detection.c \
       $(FC_ROOT)/modules/linear_drift_detection/linear_drift_detection.c \
       $(FC_ROOT)/modules/state_detector/state_detector.c \
       $(FC_ROOT)/modules/position_estimation/position_estimation.c

BUILD_DIR = build
OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(SRCS:.c=.o)))
VPATH = $(sort $(dir $(SRCS)))

CC = gcc
# Force ARM64 since we saw architecture mismatch errors with Homebrew libs
CFLAGS = -Wall -Wextra -Wno-unused-parameter -Wno-c23-extensions -I/usr/local/include -D_REENTRANT -arch arm64 \
         -I/opt/homebrew/include -I/opt/homebrew/include/SDL2 -D_THREAD_SAFE \
         -I$(FC_ROOT)/base/foundation -I$(FC_ROOT)/base/common \
         -I$(FC_ROOT)/modules \
		 -I$(FC_ROOT)/modules/rc_receiver -I$(FC_ROOT)/modules/attitude_control \
		 -I$(FC_ROOT)/modules/attitude_estimation \
		 -I$(FC_ROOT)/modules/local_storage \
		 -I$(FC_ROOT)/modules/logger \
		 -I$(FC_ROOT)/modules/fault_handler \
		 -I$(FC_ROOT)/modules/oscillation_detection \
		 -I$(FC_ROOT)/modules/linear_drift_detection \
		 -I$(FC_ROOT)/modules/position_estimation \
		 -I$(FC_ROOT)/libs/robotkit

# Build Mode Configuration (Default to Debug)
DEBUG ?= 1
ifeq ($(DEBUG), 1)
    CFLAGS += -g -DDEBUG
else
    CFLAGS += -O3 -DNDEBUG
endif

LDFLAGS = -arch arm64 -L$(FC_ROOT)/libs/robotkit -lrobotkit-macos

# SDL2 Configuration
SDL2_CFLAGS := $(shell pkg-config --cflags sdl2)
SDL2_LIBS := $(shell pkg-config --libs sdl2)

CFLAGS += $(SDL2_CFLAGS)
LDFLAGS += $(SDL2_LIBS)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
ifeq ($(DEBUG), 1)
	dsymutil $@
endif

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)
