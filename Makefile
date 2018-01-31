# This is a special makefile format for compiling kernel drivers
ifeq ($(shell uname -m),ppc64)
EXTRA_CFLAGS += -DACL_BIG_ENDIAN
endif

EXTRA_CFLAGS += -I$(PWD)/../../include

# Final module
obj-m := aclpci_drv.o

# List of object files to compile for the final module.
aclpci_drv-y := aclpci_queue.o aclpci.o aclpci_fileio.o aclpci_dma.o aclpci_cvp.o aclpci_cmd.o

