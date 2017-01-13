TARGET := vibrations_module.ko

MDIR := arch/arm/nothing

CURRENT := $(shell uname -r)
KDIR 	:= /lib/modules/$(CURRENT)/build
PWD 	:= $(shell pwd)
DEST 	:= /lib/modules/$(CURRENT)/kernel/$(MDIR)
INCL	:= -I $(KDIR)/arch/arm/include/asm/
INCL 	+= -I include/rpiGPIOManipulation.h
INCL 	+= -I include/vibration_module.h

obj-m := vibrations_module.o

default:
	$(MAKE) $(INCL) -C $(KDIR) M=$(PWD)

install:
	#@if test -f $(DEST)/$(TARGET).orig; then \
	#	echo "Backup of .ko already exists."; \
	#else \
	#	echo "Creating a backup of .ko."; \
	#	mv -v $(DEST)/$(TARGET) $(DEST)/$(TARGET).orig; \
	#fi
	su -c "cp $(TARGET) $(DEST) && /sbin/depmod -a"

revert:
	@echo "Reverting to the original .ko."
	@mv -v $(DEST)/$(TARGET).orig $(DEST)/$(TARGET)

clean:
	rm -f *.o $(TARGET) .*.cmd .*.flags *.mod.c *.symvers *.order

-include $(KDIR)/Rules.make
