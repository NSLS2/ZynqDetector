# Makefile for Linux build of GermaniumDetector.

CXX ?= g++
PKG_CONFIG ?= pkg-config

TARGET := bin/GermaniumDetector
BUILD_DIR := build

SOURCES := \
	main.cpp \
	common/Logger/Logger.cpp \
	common/DeviceWorker/DeviceWorker.cpp \
	common/AsyncWorker/AsyncWorker.cpp \
	device/Zynq/Register/Register.cpp \
	device/Zynq/PsI2c/PsI2c.cpp \
	device/Mars/Mars.cpp \
	detector/Germanium/GermaniumDetector.cpp \
	detector/Germanium/GermaniumZynq.cpp \
	detector/Germanium/GermaniumZMQ.cpp \
	detector/Germanium/GermaniumDetectorProtocol.cpp

INCLUDE_DIRS := \
	common/base \
	common/Logger \
	common/DeviceMsg \
	common/DeviceWorker \
	common/AsyncWorker \
	common/ThreadQueue \
	device/Network \
	device/Zynq \
	device/Zynq/Register \
	device/Zynq/PsI2c \
	device/Mars \
	detector/Germanium

ZMQ_CFLAGS := $(shell $(PKG_CONFIG) --cflags libzmq 2>/dev/null)
ZMQ_LIBS := $(shell $(PKG_CONFIG) --libs libzmq 2>/dev/null)

CPPFLAGS += $(addprefix -I,$(INCLUDE_DIRS)) $(ZMQ_CFLAGS)
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra
LDFLAGS ?=
LDLIBS += $(ZMQ_LIBS) -lpthread

ifeq ($(SIM_MODE),1)
CPPFLAGS += -DSIM_MODE
$(info ============ SIM_MODE enabled ============= )
endif

OBJECTS := $(addprefix $(BUILD_DIR)/,$(SOURCES:.cpp=.o))
DEPS := $(OBJECTS:.o=.d)

.PHONY: all clean run help check-deps

all: check-deps $(TARGET)

check-deps:
	@command -v $(PKG_CONFIG) >/dev/null 2>&1 || { \
		echo "Error: pkg-config is not installed."; \
		exit 1; \
	}
	@$(PKG_CONFIG) --exists libzmq || { \
		echo "Error: libzmq not found via pkg-config."; \
		echo "Install libzmq development files (and pkg-config metadata)."; \
		exit 1; \
	}

$(TARGET): $(OBJECTS) | bin
	$(CXX) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $@

bin:
	mkdir -p $@

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

run: all
	./$(TARGET)

clean:
	rm -rf $(BUILD_DIR) bin

help:
	@echo "Targets:"
	@echo "  make            Build $(TARGET)"
	@echo "  make SIM_MODE=1 Build with -DSIM_MODE"
	@echo "  make run        Run $(TARGET)"
	@echo "  make clean      Remove build artifacts"

-include $(DEPS)
