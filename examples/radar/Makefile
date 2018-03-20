CSRC = $(shell find src -name "*.c")
INCDIR = ./include

USE_PROCESS_STACKSIZE = 20480
USE_EXCEPTIONS_STACKSIZE = 20480

MODULES_ENABLED = \
chibios_sys_init \
chibios_hal_init \
app_descriptor \
boot_msg \
timing \
system \
pubsub \
worker_thread \
can_driver_stm32 \
can \
can_autobaud \
uavcan \
uavcan_nodestatus_publisher \
uavcan_getnodeinfo_server \
uavcan_beginfirmwareupdate_server \
uavcan_allocatee \
uavcan_restart \
freemem_check \
spi_device \
param \
flash \
uavcan_param_interface \
driver_dw1000 \
uavcan_debug

UDEFS += -DCAN_TX_QUEUE_LEN=128

VENDOR_DSDL_NAMESPACE_DIRS = ./dsdl/com

MESSAGES_ENABLED = \
uavcan.protocol.debug.LogMessage \
com.matternet.equipment.uwb.Radar \
com.matternet.equipment.uwb.Complex16

include ../../include.mk
