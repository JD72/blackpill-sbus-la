TOOLCHAIN_PREFIX ?= arm-none-eabi-
PYTHON ?= python3

CC      = $(TOOLCHAIN_PREFIX)gcc
OBJCOPY = $(TOOLCHAIN_PREFIX)objcopy
SIZE    = $(TOOLCHAIN_PREFIX)size

PROJECT = blackpill-sbus-la
BUILD   = build

TINYUSB_URL    = https://github.com/hathach/tinyusb
TINYUSB_COMMIT = fad6bd546fa6cb1b5fab82121cf5db436a3c38bb

TINYUSB    = lib/tinyusb
FAMILY_DIR = $(TINYUSB)/hw/bsp/stm32f4
ST_CMSIS   = $(TINYUSB)/hw/mcu/st/cmsis_device_f4
ST_HAL     = $(TINYUSB)/hw/mcu/st/stm32f4xx_hal_driver
CMSIS_CORE = $(TINYUSB)/lib/CMSIS_5/CMSIS/Core/Include

LD_FILE = board/stm32f401cc_app.ld

SRC_C = \
  src/main.c \
  src/uart1.c \
  src/cli.c \
  src/chameleon_config.c \
  src/usb_descriptors.c \
  src/usb_events.c \
  src/scpi_emul.c \
  src/log_ring.c \
  src/sampler.c \
  src/uart_dec.c \
  src/uart_gen.c \
  $(FAMILY_DIR)/family.c \
  $(TINYUSB)/src/tusb.c \
  $(TINYUSB)/src/common/tusb_fifo.c \
  $(TINYUSB)/src/device/usbd.c \
  $(TINYUSB)/src/class/usbtmc/usbtmc_device.c \
  $(TINYUSB)/src/portable/synopsys/dwc2/dcd_dwc2.c \
  $(TINYUSB)/src/portable/synopsys/dwc2/dwc2_common.c \
  $(ST_CMSIS)/Source/Templates/system_stm32f4xx.c \
  $(ST_HAL)/Src/stm32f4xx_hal.c \
  $(ST_HAL)/Src/stm32f4xx_hal_rcc.c \
  $(ST_HAL)/Src/stm32f4xx_hal_rcc_ex.c \
  $(ST_HAL)/Src/stm32f4xx_hal_gpio.c \
  $(ST_HAL)/Src/stm32f4xx_hal_flash.c \
  $(ST_HAL)/Src/stm32f4xx_hal_flash_ex.c \
  $(ST_HAL)/Src/stm32f4xx_hal_flash_ramfunc.c \
  $(ST_HAL)/Src/stm32f4xx_hal_cortex.c \
  $(ST_HAL)/Src/stm32f4xx_hal_pwr.c \
  $(ST_HAL)/Src/stm32f4xx_hal_pwr_ex.c

SRC_S = $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f401xc.s

INC = \
  -Isrc \
  -Iboard \
  -I$(TINYUSB)/src \
  -I$(TINYUSB)/hw \
  -I$(CMSIS_CORE) \
  -I$(ST_CMSIS)/Include \
  -I$(ST_HAL)/Inc

MCU_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS = \
  $(MCU_FLAGS) \
  -DSTM32F401xC -DCFG_TUSB_MCU=OPT_MCU_STM32F4 -DBOARD_TUD_RHPORT=0 \
  -Os -g -ffunction-sections -fdata-sections -fno-strict-aliasing \
  -Wall -Wextra -Wno-unused-parameter \
  $(INC)

LDFLAGS = \
  $(MCU_FLAGS) \
  -T$(LD_FILE) -nostartfiles \
  --specs=nosys.specs --specs=nano.specs \
  -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map -Wl,--print-memory-usage \
  -lgcc -lc -lm -lnosys

OBJ = $(patsubst %.c,$(BUILD)/%.o,$(SRC_C)) $(patsubst %.s,$(BUILD)/%.o,$(SRC_S))

.PHONY: all clean deps flash-dfu flash-stlink

all: $(BUILD)/$(PROJECT).bin $(BUILD)/$(PROJECT).hex
	$(SIZE) $(BUILD)/$(PROJECT).elf

deps:
	git clone --filter=blob:none $(TINYUSB_URL) $(TINYUSB)
	git -C $(TINYUSB) -c core.autocrlf=false checkout $(TINYUSB_COMMIT)
	git -C $(TINYUSB) apply ../../patches/tinyusb.patch
	cd $(TINYUSB) && $(PYTHON) tools/get_deps.py stm32f4

$(BUILD)/$(PROJECT).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU_FLAGS) -x assembler-with-cpp -c $< -o $@

clean:
	rm -rf $(BUILD)

flash-dfu: $(BUILD)/$(PROJECT).bin
	dfu-util -a 0 -s 0x08000000:leave -D $<

flash-stlink: $(BUILD)/$(PROJECT).bin
	STM32_Programmer_CLI -c port=SWD -w $< 0x08000000 -v -rst

-include $(OBJ:.o=.d)
