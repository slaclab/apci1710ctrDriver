# Location of BuildRoot home:
BUILDROOT_HOME=/afs/slac/package/linuxRT/buildroot-2019.08

# Location CrossCompiler HOME:
XCROSS_HOME=$(BUILDROOT_HOME)/host/linux-x86_64/x86_64/usr/bin/x86_64-buildroot-linux-gnu-

# Location of Kernel source:
KERNELDIR := $(BUILDROOT_HOME)/buildroot-2019.08-x86_64/output/build/linux-4.14.139

obj-m := apci1710ctr.o

all: 
# Copy source code for compiling
	cp -r ../src/*.{c,h} .
	make ARCH=x86_64 CROSS_COMPILE=$(XCROSS_HOME) -C $(KERNELDIR) M=$(PWD) modules

clean:
# Erase all files but Makefile
	find . ! -name 'Makefile' -type f -exec rm -f {} +
# Erase all directory but the root one
	find . ! -name '.' -type d -exec rm -rf {} +
