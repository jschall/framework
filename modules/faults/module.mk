FAULTS_MODULE_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

USE_PROCESS_STACKSIZE = 1024


