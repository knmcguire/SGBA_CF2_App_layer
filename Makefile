# enable app support
APP=1
APP_STACKSIZE=300

VPATH += .
PROJ_OBJ += state_machine.o
PROJ_OBJ += wallfollowing_multiranger_onboard.o
PROJ_OBJ += wallfollowing_with_avoid.o
PROJ_OBJ += SGBA.o

CRAZYFLIE_BASE= crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile
