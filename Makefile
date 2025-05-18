# Path to LLVM
LLVM_BIN_PATH := $(HOME)/Applications/LLVM/llvm-project/build/bin/
PATH := $(LLVM_BIN_PATH):$(PATH)

# LLVM Toolchain
LLVM_CC = clang
LLVM_OPT = opt
LLC = llc
LLVM_MC = llvm-mc
LLVM_LD = clang

# Target Architecture (adjust as needed)
#TARGET_ARCH_LLC = arm
#TARGET_ARCH_CC = arm-linux-gnueabihf

TARGET_ARCH_LLC = x86-64
TARGET_ARCH_CC = x86_64-linux-gnu

#TARGET_ARCH_LLC = aarch64
#TARGET_ARCH_CC = aarch64-linux-gnu

#  Registered Targets:
#    aarch64     - AArch64 (little endian)
#    aarch64_32  - AArch64 (little endian ILP32)
#    aarch64_be  - AArch64 (big endian)
#    amdgcn      - AMD GCN GPUs
#    arm         - ARM
#    arm64       - ARM64 (little endian)
#    arm64_32    - ARM64 (little endian ILP32)
#    armeb       - ARM (big endian)
#    avr         - Atmel AVR Microcontroller
#    bpf         - BPF (host endian)
#    bpfeb       - BPF (big endian)
#    bpfel       - BPF (little endian)
#    hexagon     - Hexagon
#    lanai       - Lanai
#    loongarch32 - 32-bit LoongArch
#    loongarch64 - 64-bit LoongArch
#    mips        - MIPS (32-bit big endian)
#    mips64      - MIPS (64-bit big endian)
#    mips64el    - MIPS (64-bit little endian)
#    mipsel      - MIPS (32-bit little endian)
#    msp430      - MSP430 [experimental]
#    nvptx       - NVIDIA PTX 32-bit
#    nvptx64     - NVIDIA PTX 64-bit
#    ppc32       - PowerPC 32
#    ppc32le     - PowerPC 32 LE
#    ppc64       - PowerPC 64
#    ppc64le     - PowerPC 64 LE
#    r600        - AMD GPUs HD2XXX-HD6XXX
#    riscv32     - 32-bit RISC-V
#    riscv64     - 64-bit RISC-V
#    sparc       - Sparc
#    sparcel     - Sparc LE
#    sparcv9     - Sparc V9
#    systemz     - SystemZ
#    thumb       - Thumb
#    thumbeb     - Thumb (big endian)
#    ve          - VE
#    wasm32      - WebAssembly 32-bit
#    wasm64      - WebAssembly 64-bit
#    x86         - 32-bit X86: Pentium-Pro and above
#    x86-64      - 64-bit X86: EM64T and AMD64
#    xcore       - XCore


# Flags
CFLAGS = -std=gnu99 -I/usr/local/include/ -I/usr/include/libxml2
CFLAGS += -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude
OPT_FLAGS = -O2
ASM_FLAGS =
LD_FLAGS = -no-pie # -L/usr/local/lib/$(TARGET_ARCH_CC)/
LD_LIB += -lc -lpthread -lrt -lm -lserialposix -lxml2

# Source and build directories
SRC_DIR = src
BUILD_DIR = build
LLVM_IR_DIR = $(BUILD_DIR)/llvm_ir
ASM_DIR = $(BUILD_DIR)/asm

# Common libraries
COMMON_LIBS_C = $(SRC_DIR)/rawsoc.c $(SRC_DIR)/cobs.c $(SRC_DIR)/shmbuf.c
COMMON_LIBS_LL = $(LLVM_IR_DIR)/rawsoc.ll $(LLVM_IR_DIR)/cobs.ll $(LLVM_IR_DIR)/shmbuf.ll
COMMON_LIBS_S = $(ASM_DIR)/rawsoc.s $(ASM_DIR)/cobs.s $(ASM_DIR)/shmbuf.s
COMMON_LIBS_O = $(BUILD_DIR)/rawsoc.o $(BUILD_DIR)/cobs.o $(BUILD_DIR)/shmbuf.o

# --- Target 1: ser2eth ---
TARGET1_NAME = ser2eth
TARGET1_SRC = $(SRC_DIR)/$(TARGET1_NAME).c
TARGET1_LL = $(LLVM_IR_DIR)/$(TARGET1_NAME).ll
TARGET1_S = $(ASM_DIR)/$(TARGET1_NAME).s
TARGET1_OBJ = $(BUILD_DIR)/$(TARGET1_NAME).o $(COMMON_LIBS_O)
TARGET1_OUT = $(BUILD_DIR)/$(TARGET1_NAME).out

# --- Target 2: eth2ser ---
TARGET2_NAME = eth2ser
TARGET2_SRC = $(SRC_DIR)/$(TARGET2_NAME).c
TARGET2_LL = $(LLVM_IR_DIR)/$(TARGET2_NAME).ll
TARGET2_S = $(ASM_DIR)/$(TARGET2_NAME).s
TARGET1_OBJ = $(BUILD_DIR)/$(TARGET1_NAME).o $(BUILD_DIR)/rawsoc.o $(BUILD_DIR)/cobs.o $(BUILD_DIR)/shmbuf.o
TARGET2_OBJ = $(BUILD_DIR)/$(TARGET2_NAME).o $(COMMON_LIBS_O)
TARGET2_OUT = $(BUILD_DIR)/$(TARGET2_NAME).out

# --- Target 3: serial broker ---
TARGET3_NAME = serbroker
TARGET3_SRC = $(SRC_DIR)/$(TARGET3_NAME).c
TARGET3_LL = $(LLVM_IR_DIR)/$(TARGET3_NAME).ll
TARGET3_S = $(ASM_DIR)/$(TARGET3_NAME).s
TARGET3_OBJ = $(BUILD_DIR)/$(TARGET3_NAME).o $(COMMON_LIBS_O)
TARGET3_OUT = $(BUILD_DIR)/$(TARGET3_NAME).out

# --- Target 4: launch file ---
TARGET4_NAME = launch
TARGET4_SRC = $(SRC_DIR)/$(TARGET4_NAME).c
TARGET4_LL = $(LLVM_IR_DIR)/$(TARGET4_NAME).ll
TARGET4_S = $(ASM_DIR)/$(TARGET4_NAME).s
TARGET4_OBJ = $(BUILD_DIR)/$(TARGET4_NAME).o $(COMMON_LIBS_O)
TARGET4_OUT = $(BUILD_DIR)/$(TARGET4_NAME).out

# --- Build Rules ---
# Create build directories
$(BUILD_DIR) $(LLVM_IR_DIR) $(ASM_DIR):
	@mkdir -p $@
	@echo "Created directory $@"

# Compile .c to LLVM IR (.ll)
$(LLVM_IR_DIR)/%.ll: $(SRC_DIR)/%.c | $(LLVM_IR_DIR)
	@echo "Compiling $< to LLVM IR $@"
	$(LLVM_CC) $(CFLAGS) -S -emit-llvm --target=$(TARGET_ARCH_CC) $< -o $@

# Optimize LLVM IR (optional)
$(LLVM_IR_DIR)/%.opt.ll: $(LLVM_IR_DIR)/%.ll
	@echo "Optimizing LLVM IR $< to $@"
	$(LLVM_OPT) $(OPT_FLAGS) $< -o $@

# Compile LLVM IR (.ll) to Assembly (.s)
$(ASM_DIR)/%.s: $(LLVM_IR_DIR)/%.ll | $(ASM_DIR)
	@echo "Compiling LLVM IR $< to Assembly $@"
	$(LLC) -march=$(TARGET_ARCH_LLC) $< -o $@

# Assemble Assembly (.s) to Object (.o)
$(BUILD_DIR)/%.o: $(ASM_DIR)/%.s | $(BUILD_DIR)
	@echo "Assembling $< to Object $@"
	$(LLVM_LD) $(CFLAGS) --target=$(TARGET_ARCH_CC) -c $< -o $@

# Link the object files to create the executables
$(TARGET1_OUT): $(TARGET1_OBJ)
	@echo "Linking $(TARGET1_NAME)..."
	$(LLVM_LD) --target=$(TARGET_ARCH_CC) $(LD_FLAGS) $(TARGET1_OBJ) $(LD_LIB) -o $@
	@echo "Built executable: $@"

$(TARGET2_OUT): $(TARGET2_OBJ)
	@echo "Linking $(TARGET2_NAME)..."
	$(LLVM_LD) --target=$(TARGET_ARCH_CC) $(LD_FLAGS) $(TARGET2_OBJ) $(LD_LIB) -o $@
	@echo "Built executable: $@"

$(TARGET3_OUT): $(TARGET3_OBJ)
	@echo "Linking $(TARGET3_NAME)..."
	$(LLVM_LD) --target=$(TARGET_ARCH_CC) $(LD_FLAGS) $(TARGET3_OBJ) $(LD_LIB) -o $@
	@echo "Built executable: $@"

$(TARGET4_OUT): $(TARGET4_OBJ)
	@echo "Linking $(TARGET4_NAME)..."
	$(LLVM_LD) --target=$(TARGET_ARCH_CC) $(LD_FLAGS) $(TARGET4_OBJ) $(LD_LIB) -o $@
	@echo "Built executable: $@"

# Default target to build all executables
all: $(TARGET1_OUT) $(TARGET2_OUT) $(TARGET3_OUT) $(TARGET4_OUT)
	@echo "All targets built with LLVM workflow."
	@mkdir -p ./build/bin/$(TARGET_ARCH_CC)
	@mv ./build/*.out ./build/bin/$(TARGET_ARCH_CC)

# Clean rule to remove build artifacts
clean:
	@echo "Cleaning build directory..."
	@rm -rf $(BUILD_DIR)
	@echo "Clean complete."

.PHONY: all clean