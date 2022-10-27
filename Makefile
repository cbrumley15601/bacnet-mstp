MODNAME := mstp

obj-m := $(MODNAME).o
mstp-objs := queue.o mstpmain.o
KERNEL_SRC := /usr/src/linux-headers-$(shell uname -r)

all:
	make -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	make -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean
