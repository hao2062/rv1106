KERNELDIR := /home/alientek/sdk_rv1106/rv1106_rv1103_240529/sysdrv/source/kernel
CURRENT_PATH := $(shell pwd)

obj-m := key.o

ARCH=arm
CROSS_COMPILE=/home/alientek/sdk_rv1106/rv1106_rv1103_240529/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) O=/home/alientek/sdk_rv1106/rv1106_rv1103_240529/sysdrv/source/objs_kernel M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean