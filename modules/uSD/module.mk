uSD_MODULE_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
UDEFS += -DHAL_USE_SDC -DCH_CFG_USE_SEMAPHORES=TRUE
CSRC += $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
        $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c \
        $(uSD_MODULE_DIR)/fatfs/ff.c \
        $(uSD_MODULE_DIR)/fatfs/ffunicode.c

INCDIR += $(uSD_MODULE_DIR)/fatfs

USE_PROCESS_STACKSIZE = 4096
USE_EXCEPTIONS_STACKSIZE = 4096
