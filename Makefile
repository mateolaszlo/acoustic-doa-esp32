# Makefile for acoustic-doa-esp32 off-target host testing
# Executes regression tests and feature verification without physical hardware.

CXX ?= g++
CXXFLAGS ?= -std=gnu++11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined
INCLUDES = -I test/mock_esp -I src
SRCS = test/test_adaptive_and_smoothing.cpp \
       src/audio/gcc_phat.cpp \
       src/ui/display_filter.cpp \
       src/audio/post_checker.cpp
BIN_DIR = bin
TARGET = $(BIN_DIR)/test_host

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(SRCS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRCS) -o $(TARGET)

test: $(TARGET)
	@./$(TARGET)

clean:
	@rm -rf $(BIN_DIR)
