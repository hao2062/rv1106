KERNELDIR := /home/alientek/sdk_rv1106/rv1106_rv1103_240529/sysdrv/source/objs_kernel
CURRENT_PATH := $(shell pwd)

obj-m := sc3001_acc.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean