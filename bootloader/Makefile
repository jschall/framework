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
usb_slcan

MESSAGES_ENABLED = \
uavcan.protocol.GetNodeInfo \
uavcan.protocol.file.BeginFirmwareUpdate \
uavcan.protocol.file.Read \
uavcan.protocol.RestartNode \
uavcan.equipment.indication.LightsCommand \
com.hex.file.FileStreamStart \
com.hex.file.FileStreamChunk

DSDL_NAMESPACE_DIRS += dsdl/com

LOAD_REGION = bl

UDEFS += -DTARGET_BOOTLOADER

include ../include.mk

upload: build/$(BOARD)/$(PROJECT).elf
	openocd -f openocd.cfg -c "program $< verify reset exit"
