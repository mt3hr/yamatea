mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
IsPrintMessage.o \
PrintMessage.o \
util.o \
Preparizable.o \
Command.o \
Predicate.o \
NumberOfTimesPredicate.o \
CommandAndPredicate.o \
WheelController.o \
MotorCountPredicate.o \
StartButtonPredicate.o \
CommandExecutor.o \
RotationRobot.o \
SuitableForRightCourse.o \
PIDTargetBrightnessCalibrator.o \
PIDTracer.o \
Walker.o \
Handler.o \
SetPIDTargetBrightnessWhenCalibratedHandler.o \
RGBRawReader.o \
DistanceReader.o \
ArmController.o \
ExecutePreparationWhenExitBeforeCommandHandler.o \
MotorRotationAnglePredicate.o \
Stopper.o \
ObstacleDetectRunner.o \
UFORunner.o \
AwaitingSignalRunner.o \
StraightBetweenRunner.o \
SuperSocialDistanceRunner.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += \
	$(mkfile_path)command \
	$(mkfile_path)handler \
	$(mkfile_path)predicate \
	$(mkfile_path)robot \
	$(mkfile_path)sample

INCLUDES += \
	-I$(mkfile_path)command \
	-I$(mkfile_path)handler \
	-I$(mkfile_path)predicate \
	-I$(mkfile_path)robot \
	-I$(mkfile_path)sample

#COPTS += -fno-use-cxa-atexit

COPTS += -DMAKE_EV3
