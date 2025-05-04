# Compiler and flags
CC = gcc -std=gnu17
CFLAGS = -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude 
LDFLAGS = -pthread -lrt -lm -lserialposix

# Source and build directories
SRC_DIR = src
BUILD_DIR = build

# Common libraries used by both executables
COMMON_LIBS_C = $(SRC_DIR)/rawsoc.c $(SRC_DIR)/cobs.c $(SRC_DIR)/shmbuf.c
COMMON_LIBS_O = $(BUILD_DIR)/rawsoc.o $(BUILD_DIR)/cobs.o $(BUILD_DIR)/shmbuf.o

# --- Target 1: ser2eth ---
TARGET1_NAME = ser2eth
TARGET1_SRC = $(SRC_DIR)/$(TARGET1_NAME).c
TARGET1_OBJ = $(BUILD_DIR)/$(TARGET1_NAME).o $(COMMON_LIBS_O)

# --- Target 2: eth2ser ---
TARGET2_NAME = eth2ser
TARGET2_SRC = $(SRC_DIR)/$(TARGET2_NAME).c
TARGET2_OBJ = $(BUILD_DIR)/$(TARGET2_NAME).o $(COMMON_LIBS_O)

# --- Target 3: test ---
TARGET3_NAME = tstdriver
TARGET3_SRC = $(SRC_DIR)/$(TARGET3_NAME).c
TARGET3_OBJ = $(BUILD_DIR)/$(TARGET3_NAME).o $(COMMON_LIBS_O)

# --- Build Rules ---
# Create the build directory if it doesn't exist
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)
	@echo "Created directory $(BUILD_DIR)"

# Pattern rule to compile .c files to .o files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo "Compiling $< to $@"
	$(CC) $(CFLAGS) -c $< -o $@

# Link the object files to create the executables
$(BUILD_DIR)/$(TARGET1_NAME).out: $(TARGET1_OBJ)
	@echo "Linking $(TARGET1_NAME)..."
	$(CC) $(TARGET1_OBJ) $(LDFLAGS) -o $@
	@echo "Built executable: $@"

$(BUILD_DIR)/$(TARGET2_NAME).out: $(TARGET2_OBJ)
	@echo "Linking $(TARGET2_NAME)..."
	$(CC) $(TARGET2_OBJ) $(LDFLAGS) -o $@
	@echo "Built executable: $@"

$(BUILD_DIR)/$(TARGET3_NAME).out: $(TARGET3_OBJ)
	@echo "Linking $(TARGET3_NAME)..."
	$(CC) $(TARGET3_OBJ) $(LDFLAGS) -o $@
	@echo "Built executable: $@"

# Default target to build all executables
all: $(BUILD_DIR)/$(TARGET1_NAME).out $(BUILD_DIR)/$(TARGET2_NAME).out $(BUILD_DIR)/$(TARGET3_NAME).out
	@echo "All targets built."

# Clean rule to remove build artifacts
clean:
	@echo "Cleaning build directory..."
	@rm -rf $(BUILD_DIR)
	@echo "Clean complete."