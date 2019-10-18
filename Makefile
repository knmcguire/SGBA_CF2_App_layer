# enable app support
APP=1
APP_STACKSIZE=300

VPATH += .
PROJ_OBJ += state_machine.o

CRAZYFLIE_BASE= crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile
