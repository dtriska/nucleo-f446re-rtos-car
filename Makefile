##########################################################################################################################
# Minimal CMSIS-only Makefile for STM32F446RE (Nucleo)
# Keeps: startup + linker + CMSIS. No HAL, no CubeMX required.
##########################################################################################################################

######################################
# target
######################################
TARGET = nucleo_f446re_blinky

######################################
# build options
######################################
DEBUG = 1
OPT   = -Og

######################################
# paths
######################################
BUILD_DIR = build

######################################
# source (match your *current* repo)
######################################
C_SOURCES = \
Core/Src/main.c \
Core/Src/system_stm32f4xx.c \
Core/Src/stm32f4xx_it.c \
Core/Src/newlib_stubs.c \
Core/Src/sysmem.c \
Core/Src/gpio.c \
Core/Src/uart.c \
Core/Src/i2c.c \
Core/Src/i2c_int.c

ASM_SOURCES = \
startup_stm32f446xx.s

######################################
# toolchain
######################################
PREFIX = arm-none-eabi-

ifdef GCC_PATH
CC  = $(GCC_PATH)/$(PREFIX)gcc
AS  = $(GCC_PATH)/$(PREFIX)gcc
CP  = $(GCC_PATH)/$(PREFIX)objcopy
SZ  = $(GCC_PATH)/$(PREFIX)size
OBJDUMP = $(GCC_PATH)/$(PREFIX)objdump
else
CC  = $(PREFIX)gcc
AS  = $(PREFIX)gcc
CP  = $(PREFIX)objcopy
SZ  = $(PREFIX)size
OBJDUMP = $(PREFIX)objdump
endif

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

######################################
# MCU flags
######################################
CPU       = -mcpu=cortex-m4
FPU       = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard
MCU       = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

######################################
# Defines / Includes (CMSIS only)
######################################
C_DEFS = \
-DSTM32F446xx

C_INCLUDES = \
-ICore/Inc \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include

######################################
# Compile flags
######################################
ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections -x assembler-with-cpp

CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Wextra \
          -fdata-sections -ffunction-sections

ifeq ($(DEBUG),1)
CFLAGS += -g -gdwarf-2
endif

# Dependency generation
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

######################################
# Linker flags
######################################
LDSCRIPT = STM32F446XX_FLASH.ld

# Use your startup file; don't let gcc pull default crt0.
# Keep nosys to avoid syscall headaches early.
LIBS   = -lc -lm -lnosys
LDFLAGS = $(MCU) -nostartfiles -specs=nano.specs -T$(LDSCRIPT) $(LIBS) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
          -Wl,--gc-sections \
          -Wl,--print-memory-usage

######################################
# build products
######################################
ELF = $(BUILD_DIR)/$(TARGET).elf
HEX_OUT = $(BUILD_DIR)/$(TARGET).hex
BIN_OUT = $(BUILD_DIR)/$(TARGET).bin

######################################
# objects
######################################
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

######################################
# default target
######################################
.PHONY: all clean flash size disasm

all: $(ELF) $(HEX_OUT) $(BIN_OUT)

$(BUILD_DIR):
	mkdir -p $@

######################################
# compile rules
######################################
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

######################################
# link rules
######################################
$(ELF): $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(HEX_OUT): $(ELF) | $(BUILD_DIR)
	$(HEX) $< $@

$(BIN_OUT): $(ELF) | $(BUILD_DIR)
	$(BIN) $< $@

######################################
# convenience targets
######################################
clean:
	-rm -rf $(BUILD_DIR)

size: $(ELF)
	$(SZ) $(ELF)

disasm: $(ELF)
	$(OBJDUMP) -d $(ELF) > $(BUILD_DIR)/$(TARGET).disasm.S
	@echo "Wrote $(BUILD_DIR)/$(TARGET).disasm.S"

# Optional: ST-LINK flash (works if st-flash is installed)
flash: $(BIN_OUT)
	st-flash write $(BIN_OUT) 0x08000000

# deps
-include $(wildcard $(BUILD_DIR)/*.d)

