CSRC = $(shell find src -name "*.c")
INCDIR = ./include

MODULES_ENABLED = \
chibios_sys_init \
chibios_hal_init \
app_descriptor \
boot_msg \
timing \
system \
pubsub \
param \
flash \
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
uavcan_timesync \
uavcan_debug \
faults \
uSD \
logger

UDEFS += -DHAL_USE_GPT=TRUE -DSTM32_GPT_USE_TIM3=TRUE -DMICROSD_MOUNT_FAIL_REFORMAT

# UAVCAN DSDL messages required by uavcan_debug and logger debug
MESSAGES_ENABLED = \
uavcan.protocol.debug.LogMessage \
uavcan.protocol.debug.KeyValue


include ../../include.mk
