CSRC = $(shell find src -name "*.c")
EXCLUDE_OBJS += main.o
INCDIR = ./include

# Enable as many modules as possible on STM32F427
MODULES_ENABLED = \
chibios_sys_init \
chibios_hal_init \
app_descriptor \
boot_msg \
timing \
system \
system_event \
pubsub \
pubsub_miss_measurement \
param \
flash \
worker_thread \
load_measurement \
stack_measurement \
can_driver_stm32 \
can \
can_autobaud \
can_terminator \
uavcan \
uavcan_nodestatus_publisher \
uavcan_getnodeinfo_server \
uavcan_beginfirmwareupdate_server \
uavcan_allocatee \
uavcan_restart \
uavcan_timesync \
uavcan_broadcast_file_update \
uavcan_file_server \
uavcan_filesystem_util \
uavcan_node_registry \
uavcan_param_interface \
 uavcan_debug \
 usb_slcan \
 datalogger \
 #pin_change_publisher \
spi_device \
sensor_registry \
sensor_imu_uavcan_in \
sensor_mag_uavcan_out \
sensor_gnss_uavcan_out

# Filesystem / uSD support is included by default; boards that don't support it
# should filter it out in their board.mk.
MODULES_ENABLED += uSD

# Do not force-enable HAL I2C/SPI to avoid missing peripheral assignments.

# Messages used by the enabled UAVCAN modules
MESSAGES_ENABLED = \
uavcan.protocol.debug.LogMessage \
uavcan.protocol.NodeStatus \
uavcan.protocol.GetNodeInfo \
uavcan.protocol.file.BeginFirmwareUpdate \
uavcan.protocol.file.Read \
uavcan.protocol.RestartNode \
uavcan.protocol.param.GetSet \
uavcan.protocol.param.ExecuteOpcode \
uavcan.equipment.indication.LightsCommand \
uavcan.protocol.file.Write \
uavcan.protocol.dynamic_node_id.Allocation \
 uavcan.protocol.GlobalTimeSync \
 uavcan.equipment.gnss.Fix2 \
 uavcan.equipment.gnss.Auxiliary \
 uavcan.equipment.ahrs.MagneticFieldStrength \
 com.hex.file.FileStreamChunk

# No vendor DSDL used in this app; use core UAVCAN set only.

include ../../include.mk

