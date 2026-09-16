# SPDX-License-Identifier: GPL-2.0


# Updated for SPSC modules
obj-m += bmi_spsc.o

# BMI088 driver with SPSC integration
bmi088-objs := bmi088_core.o
obj-m += bmi088.o

ccflags-y += -I$(src)/../../../out/nvidia-conftest/
ccflags-y += -I$(TEGRA_KERNEL_OUT)/out/nvidia-conftest/

# Build configuration
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# Default target
all: modules

# Build modules
modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean build artifacts
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f *.ur-safe

.PHONY: all modules clean