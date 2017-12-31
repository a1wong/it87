# For building for the current running version of Linux
TARGET		:= $(shell uname -r)
# Or specific version
#TARGET		:= 2.6.33.5
KERNEL_MODULES	:= /lib/modules/$(TARGET)

ifneq ("","$(wildcard /usr/src/linux-headers-$(TARGET)/*)")
# Ubuntu
KERNEL_BUILD	:= /usr/src/linux-headers-$(TARGET)
else
ifneq ("","$(wildcard /usr/src/kernels/$(TARGET)/*)")
# Fedora
KERNEL_BUILD	:= /usr/src/kernels/$(TARGET)
else
KERNEL_BUILD	:= $(KERNEL_MODULES)/build
endif
endif

#SYSTEM_MAP	:= $(KERNEL_BUILD)/System.map
SYSTEM_MAP	:= /boot/System.map-$(TARGET)

DRIVER := it87

# Directory below /lib/modules/$(TARGET)/kernel into which to install
# the module:
MOD_SUBDIR = drivers/hwmon
MODDESTDIR=$(KERNEL_MODULES)/kernel/$(MOD_SUBDIR)

obj-m	:= $(patsubst %,%.o,$(DRIVER))
obj-ko  := $(patsubst %,%.ko,$(DRIVER))

MAKEFLAGS += --no-print-directory

ifneq ("","$(wildcard $(MODDESTDIR)/*.ko.gz)")
COMPRESS_GZIP := y
endif
ifneq ("","$(wildcard $(MODDESTDIR)/*.ko.xz)")
COMPRESS_XZ := y
endif

.PHONY: all install modules modules_install clean

all: modules

# Targets for running make directly in the external module directory:

ifneq ("","$(wildcard .git/*)")
IT87_CFLAGS=-DIT87_DRIVER_VERSION='\"$(shell git describe --long)\"'
else
IT87_CFLAGS=-DIT87_DRIVER_VERSION='\"<unknown>\"'
endif

modules:
	@$(MAKE) EXTRA_CFLAGS="$(IT87_CFLAGS)" -C $(KERNEL_BUILD) M=$(CURDIR) $@

clean:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(CURDIR) $@

install: modules_install

modules_install:
	mkdir -p $(MODDESTDIR)
	cp $(DRIVER).ko $(MODDESTDIR)/
ifeq ($(COMPRESS_GZIP), y)
	@gzip -f $(MODDESTDIR)/$(DRIVER).ko
endif
ifeq ($(COMPRESS_XZ), y)
	@xz -f $(MODDESTDIR)/$(DRIVER).ko
endif
	depmod -a -F $(SYSTEM_MAP) $(TARGET)
