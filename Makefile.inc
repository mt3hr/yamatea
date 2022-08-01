mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

# APPL_COBJS +=

APPL_CXXOBJS += \
util.o \
Command.o \
Predicate.o \
MotorCountPredicate.o \
StartButtonPredicate.o \
CommandExecutor.o \
PIDTargetBrightnessCalibrator.o \
PIDTracer.o \
ScenarioTracer.o \
Handler.o \
SetPIDTargetBrightnessWhenCalibratedHandler.o \
# SwingDistanceCompensationHandler.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += $(mkfile_path)app

INCLUDES += \
	-I$(mkfile_path)app

#COPTS += -fno-use-cxa-atexit

COPTS += -DMAKE_EV3
