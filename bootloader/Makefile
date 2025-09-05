CSRC = $(shell find src -name "*.c")
INCDIR = ./include

MODULE_SEARCH_DIRS = modules
USE_PROCESS_STACKSIZE = 2048
USE_EXCEPTIONS_STACKSIZE = 2048

MODULES_ENABLED = \
chibios_sys_init \
chibios_hal_init \
boot_msg \
timing \
system \
pubsub \
worker_thread \
flash \
param \
uavcan_param_interface \
can_driver_stm32 \
can \
can_autobaud \
uavcan \
uavcan_nodestatus_publisher \
uavcan_allocatee \
uavcan_debug \
usb_slcan \
pubsub_miss_measurement \
load_measurement

MESSAGES_ENABLED = \
uavcan.protocol.GetNodeInfo \
uavcan.protocol.file.BeginFirmwareUpdate \
uavcan.protocol.file.Read \
uavcan.protocol.RestartNode \
uavcan.protocol.debug.LogMessage \
uavcan.protocol.debug.KeyValue \
uavcan.equipment.indication.LightsCommand \
com.hex.file.FileStreamStart \
com.hex.file.FileStreamChunk

DSDL_NAMESPACE_DIRS += dsdl/com

LOAD_REGION = bl

UDEFS += -DTARGET_BOOTLOADER

include ../include.mk

# DFU (USB Device Firmware Upgrade) upload settings
DFU_DEVICE ?= 0483:df11
DFU_ALT ?= 0
DFU_ADDR ?= 0x08000000

.PHONY: upload dfu mass_erase

upload: build/$(BOARD)/$(PROJECT).elf
	openocd -f openocd.cfg -c "program $< verify reset exit"

dfu-upload: build/$(BOARD)/$(PROJECT).bin
	dfu-util -d $(DFU_DEVICE) -a $(DFU_ALT) -s $(DFU_ADDR):leave -D $<

mass_erase:
	openocd -f openocd.cfg -c "init;halt;stm32h7x mass_erase 0;exit"
