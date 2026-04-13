###############################################################################
# STM32F411CEU6 Profibus DP Slave - Makefile
# Bare-metal build with arm-none-eabi-gcc
#
# Usage:
#   make          - Build firmware (.elf, .bin, .hex)
#   make clean    - Remove build artifacts
#   make flash    - Flash via ST-Link (requires st-flash)
#   make flash-dfu - Flash via USB DFU (requires dfu-util)
#   make size     - Show firmware size
###############################################################################

# Project name
TARGET = stm32f411_profibus

# Toolchain
PREFIX = arm-none-eabi-
CC     = $(PREFIX)gcc
AS     = $(PREFIX)gcc -x assembler-with-cpp
CP     = $(PREFIX)objcopy
SZ     = $(PREFIX)size
GDB    = $(PREFIX)gdb

# MCU flags
CPU    = -mcpu=cortex-m4
FPU    = -mfloat-abi=soft
MCU    = $(CPU) -mthumb $(FPU)

# Sources
C_SOURCES = main.c
ASM_SOURCES = stm32f411_startup.s

# Linker script
LDSCRIPT = STM32F411CEUx_FLASH.ld

# Build directory
BUILD_DIR = build

# Compiler flags
CFLAGS  = $(MCU)
CFLAGS += -std=c99
CFLAGS += -Wall -Wextra -Wpedantic
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -fno-common
CFLAGS += -Os
# Debug info (remove -g for release)
CFLAGS += -g

# Assembler flags
ASFLAGS = $(MCU)
ASFLAGS += -Wall

# Linker flags
LDFLAGS  = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += --specs=nosys.specs
LDFLAGS += --specs=nano.specs
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -lnosys

###############################################################################
# Build rules
###############################################################################

# Object files
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex size

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C sources
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble .s files
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Create binary
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary $< $@

# Create Intel HEX
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex $< $@

# Show size
size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

# Flash via ST-Link
flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $< 0x08000000

# Flash via DFU (built-in USB bootloader)
# Hold BOOT0 button while pressing RESET to enter DFU mode
flash-dfu: $(BUILD_DIR)/$(TARGET).bin
	dfu-util -a 0 -s 0x08000000:leave -D $<

# Flash via OpenOCD + ST-Link
flash-openocd: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	  -c "program $< verify reset exit"

# Debug with GDB + OpenOCD
debug: $(BUILD_DIR)/$(TARGET).elf
	$(GDB) -ex "target remote :3333" -ex "monitor reset halt" $<

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Phony targets
.PHONY: all clean flash flash-dfu flash-openocd debug size
