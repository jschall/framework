LOGGER_MODULE_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
UDEFS += -DHAL_USE_SDC=1
CSRC += $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
        $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c \
        $(LOGGER_MODULE_DIR)/fatfs/ff.c \
        $(LOGGER_MODULE_DIR)/fatfs/ffunicode.c

INCDIR += $(LOGGER_MODULE_DIR)/fatfs

USE_PROCESS_STACKSIZE = 4096
USE_EXCEPTIONS_STACKSIZE = 4096
