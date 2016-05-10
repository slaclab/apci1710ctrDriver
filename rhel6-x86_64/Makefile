# Location of Kernel source:
KERNELDIR := /lib/modules/$(shell uname -r)/build

obj-m := apci1710ctr.o

#apci1710ctr-objs := apci1710ctr.o

all: 
# Copy source code for compiling
	cp -r ../src/*.{c,h} .
	make -C $(KERNELDIR) M=$(PWD) modules

clean:
# Erase all files but Makefile
	find . ! -name 'Makefile' -type f -exec rm -f {} +
# Erase all directory but the root one
	find . ! -name '.' -type d -exec rm -rf {} +
