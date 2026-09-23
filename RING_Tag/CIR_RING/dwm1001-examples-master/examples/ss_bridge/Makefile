###############################################################################
# Makefile for ss_bridge (DWM1001 TWR Bridge)
# Converted from SES emProject configuration
###############################################################################

# Project name
PROJECT_NAME := ss_bridge

# Paths relative to this Makefile (examples/ss_bridge/)
SDK_ROOT     := ../../nRF5_SDK_14.2.0
DECA_DRIVER  := ../../deca_driver
BOARDS_DIR   := ../../boards
PROJ_DIR     := .

# Output
OUTPUT_DIRECTORY := _build
TARGETS          := $(PROJECT_NAME)

# Toolchain - auto-detect arm-none-eabi-gcc in PATH
GNU_PREFIX  := arm-none-eabi
CC          := $(GNU_PREFIX)-gcc
CXX         := $(GNU_PREFIX)-c++
AS          := $(GNU_PREFIX)-gcc -x assembler-with-cpp
LD          := $(GNU_PREFIX)-gcc
OBJCOPY     := $(GNU_PREFIX)-objcopy
OBJDUMP     := $(GNU_PREFIX)-objdump
SIZE        := $(GNU_PREFIX)-size

# Linker script
LINKER_SCRIPT := $(SDK_ROOT)/components/toolchain/gcc/nrf52_xxaa.ld

# Source files - Application
SRC_FILES += $(PROJ_DIR)/main.c
SRC_FILES += $(PROJ_DIR)/ss_init_main.c
SRC_FILES += $(PROJ_DIR)/UART/UART.c

# Source files - nRF system
SRC_FILES += $(SDK_ROOT)/components/toolchain/system_nrf52.c
SRC_FILES += $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.S

# Source files - Board
SRC_FILES += $(SDK_ROOT)/components/boards/boards.c
SRC_FILES += $(SDK_ROOT)/components/libraries/bsp/bsp.c
SRC_FILES += $(SDK_ROOT)/components/libraries/bsp/bsp_nfc.c

# Source files - nRF Drivers
SRC_FILES += $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c
SRC_FILES += $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c
SRC_FILES += $(SDK_ROOT)/components/drivers_nrf/spi_master/nrf_drv_spi.c
SRC_FILES += $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c
SRC_FILES += $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c

# Source files - nRF Libraries
SRC_FILES += $(SDK_ROOT)/components/libraries/button/app_button.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/app_error.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/app_error_weak.c
SRC_FILES += $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/app_util_platform.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/nrf_assert.c
SRC_FILES += $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c
SRC_FILES += $(SDK_ROOT)/external/fprintf/nrf_fprintf.c
SRC_FILES += $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_memobj/nrf_memobj.c
SRC_FILES += $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c
SRC_FILES += $(SDK_ROOT)/components/libraries/fifo/app_fifo.c
SRC_FILES += $(SDK_ROOT)/components/libraries/uart/app_uart_fifo.c
SRC_FILES += $(SDK_ROOT)/components/libraries/uart/retarget.c

# Source files - nRF Log
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_rtt.c
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_serial.c
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_uart.c
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_default_backends.c
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_frontend.c
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_str_formatter.c

# Source files - Segger RTT
SRC_FILES += $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c
SRC_FILES += $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c

# Source files - Decawave driver
SRC_FILES += $(DECA_DRIVER)/deca_device.c
SRC_FILES += $(DECA_DRIVER)/deca_params_init.c
SRC_FILES += $(DECA_DRIVER)/port/port_platform.c

# Source files - FreeRTOS
SRC_FILES += $(SDK_ROOT)/external/freertos/source/croutine.c
SRC_FILES += $(SDK_ROOT)/external/freertos/source/event_groups.c
SRC_FILES += $(SDK_ROOT)/external/freertos/source/list.c
SRC_FILES += $(SDK_ROOT)/external/freertos/source/queue.c
SRC_FILES += $(SDK_ROOT)/external/freertos/source/tasks.c
SRC_FILES += $(SDK_ROOT)/external/freertos/source/timers.c
SRC_FILES += $(SDK_ROOT)/external/freertos/source/portable/MemMang/heap_1.c
SRC_FILES += $(SDK_ROOT)/components/libraries/timer/app_timer_freertos.c
SRC_FILES += $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52/port_cmsis.c
SRC_FILES += $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52/port_cmsis_systick.c
SRC_FILES += $(SDK_ROOT)/external/freertos/portable/GCC/nrf52/port.c

###############################################################################
# Include paths
###############################################################################
INC_FOLDERS += $(PROJ_DIR)/config
INC_FOLDERS += $(PROJ_DIR)/UART
INC_FOLDERS += $(BOARDS_DIR)
INC_FOLDERS += $(DECA_DRIVER)
INC_FOLDERS += $(DECA_DRIVER)/port
INC_FOLDERS += $(SDK_ROOT)/components
INC_FOLDERS += $(SDK_ROOT)/components/boards
INC_FOLDERS += $(SDK_ROOT)/components/device
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/clock
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/common
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/delay
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/gpiote
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/hal
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/spi_master
INC_FOLDERS += $(SDK_ROOT)/components/drivers_nrf/uart
INC_FOLDERS += $(SDK_ROOT)/components/libraries/atomic
INC_FOLDERS += $(SDK_ROOT)/components/libraries/balloc
INC_FOLDERS += $(SDK_ROOT)/components/libraries/bsp
INC_FOLDERS += $(SDK_ROOT)/components/libraries/button
INC_FOLDERS += $(SDK_ROOT)/components/libraries/fifo
INC_FOLDERS += $(SDK_ROOT)/components/libraries/experimental_log
INC_FOLDERS += $(SDK_ROOT)/components/libraries/experimental_log/src
INC_FOLDERS += $(SDK_ROOT)/components/libraries/experimental_memobj
INC_FOLDERS += $(SDK_ROOT)/components/libraries/experimental_section_vars
INC_FOLDERS += $(SDK_ROOT)/components/libraries/scheduler
INC_FOLDERS += $(SDK_ROOT)/components/libraries/strerror
INC_FOLDERS += $(SDK_ROOT)/components/libraries/timer
INC_FOLDERS += $(SDK_ROOT)/components/libraries/util
INC_FOLDERS += $(SDK_ROOT)/components/libraries/uart
INC_FOLDERS += $(SDK_ROOT)/components/toolchain
INC_FOLDERS += $(SDK_ROOT)/components/toolchain/cmsis/include
INC_FOLDERS += $(SDK_ROOT)/external/fprintf
INC_FOLDERS += $(SDK_ROOT)/external/segger_rtt
INC_FOLDERS += $(SDK_ROOT)/external/freertos
INC_FOLDERS += $(SDK_ROOT)/external/freertos/source
INC_FOLDERS += $(SDK_ROOT)/external/freertos/config
INC_FOLDERS += $(SDK_ROOT)/external/freertos/source/include
INC_FOLDERS += $(SDK_ROOT)/external/freertos/portable/ARM/nrf52
INC_FOLDERS += $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52
INC_FOLDERS += $(SDK_ROOT)/external/freertos/source/portable
INC_FOLDERS += ../..

###############################################################################
# Flags
###############################################################################

# Preprocessor defines (from emProject c_preprocessor_definitions)
CDEFINES += -DBOARD_DW1001_DEV
CDEFINES += -DBSP_SIMPLE
CDEFINES += -DCONFIG_GPIO_AS_PINRESET
CDEFINES += -DFLOAT_ABI_HARD
CDEFINES += -DNRF52
CDEFINES += -DNRF52832_XXAA
CDEFINES += -DNRF52_PAN_74
CDEFINES += -DSWI_DISABLE0
CDEFINES += -D__HEAP_SIZE=8192
CDEFINES += -D__STACK_SIZE=8192

# C flags
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb
CFLAGS += -mabi=aapcs
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=fpv4-sp-d16
CFLAGS += -Wall
CFLAGS += -O3 -g3
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -fno-strict-aliasing
CFLAGS += -fno-builtin
CFLAGS += -fshort-enums
CFLAGS += -std=c99
CFLAGS += $(CDEFINES)

# Assembler flags
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb
ASMFLAGS += -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard
ASMFLAGS += -mfpu=fpv4-sp-d16
ASMFLAGS += $(CDEFINES)

# Linker flags
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mthumb
LDFLAGS += -mabi=aapcs
LDFLAGS += -mfloat-abi=hard
LDFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs
LDFLAGS += -u _printf_float
LDFLAGS += -L$(SDK_ROOT)/components/toolchain/gcc

###############################################################################
# Build rules
###############################################################################

# Collect object files
C_SRC   = $(filter %.c, $(SRC_FILES))
ASM_SRC = $(filter %.S, $(SRC_FILES))
C_OBJ   = $(C_SRC:%.c=$(OUTPUT_DIRECTORY)/%.o)
ASM_OBJ = $(ASM_SRC:%.S=$(OUTPUT_DIRECTORY)/%.o)
OBJ     = $(C_OBJ) $(ASM_OBJ)
DEPS    = $(OBJ:.o=.d)

INC_FLAGS = $(addprefix -I, $(INC_FOLDERS))

.PHONY: all clean flash flash_erase

all: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).bin
	@echo ""
	@echo "===== Build complete: $(PROJECT_NAME) ====="
	@echo "  HEX: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex"
	@echo "  BIN: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).bin"

# Link
$(OUTPUT_DIRECTORY)/$(PROJECT_NAME).out: $(OBJ)
	@echo "Linking: $@"
	$(LD) $(LDFLAGS) $(OBJ) -Wl,-Map=$(@:.out=.map) -o $@
	$(SIZE) $@

# Create HEX
$(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).out
	@echo "Creating HEX: $@"
	$(OBJCOPY) -O ihex $< $@

# Create BIN
$(OUTPUT_DIRECTORY)/$(PROJECT_NAME).bin: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).out
	@echo "Creating BIN: $@"
	$(OBJCOPY) -O binary $< $@

# Compile C files
$(OUTPUT_DIRECTORY)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling: $(notdir $<)"
	$(CC) $(CFLAGS) $(INC_FLAGS) -MP -MD -c -o $@ $<

# Compile ASM files
$(OUTPUT_DIRECTORY)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo "Assembling: $(notdir $<)"
	$(CC) $(ASMFLAGS) $(INC_FLAGS) -MP -MD -c -o $@ $<

# Include dependency files
-include $(DEPS)

###############################################################################
# Flash targets (using nrfjprog / J-Link)
###############################################################################

# Flash using nrfjprog (Nordic command line tools)
flash: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex
	@echo "Flashing: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex"
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex --sectorerase
	nrfjprog -f nrf52 --reset

# Erase all flash
flash_erase:
	nrfjprog -f nrf52 --eraseall

# Flash using J-Link Commander directly (alternative)
flash_jlink: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex
	@echo "device nRF52832_xxAA"  > $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "si SWD"               >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "speed 4000"           >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "connect"              >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "erase"                >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "loadfile $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex" >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "r"                    >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "g"                    >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	@echo "exit"                 >> $(OUTPUT_DIRECTORY)/jlink_flash.jlink
	JLinkExe -CommandFile $(OUTPUT_DIRECTORY)/jlink_flash.jlink

# Clean build
clean:
	rm -rf $(OUTPUT_DIRECTORY)
