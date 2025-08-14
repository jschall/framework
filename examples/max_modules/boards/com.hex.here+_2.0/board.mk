BOARD_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
BOARD_SRC = $(BOARD_DIR)/board.c
BOARD_INC = $(BOARD_DIR)
MODULES_ENABLED += platform_stm32f302x8
# Drop modules that require USB/uSD on F3
MODULES_ENABLED := $(filter-out usb_slcan uSD datalogger uavcan_file_server uavcan_filesystem_util uavcan_broadcast_file_update,$(MODULES_ENABLED))
