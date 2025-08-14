BOARD_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
BOARD_SRC = $(BOARD_DIR)/board.c
BOARD_INC = $(BOARD_DIR)
MODULES_ENABLED += platform_stm32h743
# H7: keep maximum modules (USB/uSD supported)
