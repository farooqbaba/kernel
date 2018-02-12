#
#This software is contributed or developed by KYOCERA Corporation.
#(C) 2013 KYOCERA Corporation
#

MODULES = EP957_Driver

obj-$(CONFIG_EP957B_MHL) += $(MODULES).o

$(MODULES)-objs := \
                   EP957_If.o \
                   EP957Controller.o

clean:
	rm -f .*.o.*
