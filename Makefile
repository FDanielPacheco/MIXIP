CC = gcc -std=gnu17

CFLAGS = -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude 
LDFLAGS = -pthread -lrt -lm -lserialposix

NAME = dualcom
BUILD_DIR = build
SRC_DIR = src

SRCS = $(SRC_DIR)/$(NAME).c $(SRC_DIR)/cobs.c $(SRC_DIR)/ringbuffer.c
OBJS = $(BUILD_DIR)/$(NAME).o $(BUILD_DIR)/cobs.o $(BUILD_DIR)/ringbuffer.o

# Rule to create the build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compilation rules
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Linking rule
$(BUILD_DIR)/$(NAME).out: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

# Default target
all: $(BUILD_DIR)/$(NAME).out

# Clean build directory
clean:
	rm -rf $(BUILD_DIR)
