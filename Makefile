# make KERNEL_SRC=/home/lelle/sdk/vec6200/sysroots/armv7at2hf-neon-oe-linux-gnueabi/usr/src/kernel && scp vmcu.ko dtest:/tmp/ && ssh dtest "insmod /tmp/vmcu.ko && rmmod vmcu"

obj-m := vmcu.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod *.a
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers