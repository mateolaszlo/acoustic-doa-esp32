# Makefile for acoustic-doa-esp32 off-target host testing
# Executes regression tests and feature verification without physical hardware.

CXX ?= g++

# Base compilation standards and warnings
BASE_FLAGS = -std=gnu++11 -Wall -Wextra -Werror

# Expanded Sanitizer Configuration:
# - address: Heap, stack, global buffer overflows, double-free, leaks
# - undefined: Integer overflow, shift bounds, alignment, float cast overflow
# - float-divide-by-zero: Traps silent NaN/Inf propagation in DSP pipelines
# - fno-sanitize-recover=all: Guarantees CI fails immediately on UBSan errors
# - -g -fno-omit-frame-pointer: Human-readable stack traces with exact line numbers
# - -O1: Keeps runtime assertions intact without aggressive -O2 dead-code elimination
SAN_FLAGS = -fsanitize=address,undefined,float-divide-by-zero \
            -fno-sanitize-recover=all \
            -fno-omit-frame-pointer \
            -g -O1

CXXFLAGS ?= $(BASE_FLAGS) $(SAN_FLAGS)
INCLUDES  = -I test/mock_esp -I src
SRCS      = test/test_adaptive_and_smoothing.cpp \
            src/audio/gcc_phat.cpp \
            src/ui/display_filter.cpp \
            src/audio/post_checker.cpp \
            src/ui/radar_ui.cpp
BIN_DIR   = bin
TARGET    = $(BIN_DIR)/test_host

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(SRCS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRCS) -o $(TARGET)

test: $(TARGET)
	@ASAN_OPTIONS=detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1 \
	 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
	 ./$(TARGET)

clean:
	@rm -rf $(BIN_DIR)
