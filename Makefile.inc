mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
Setting.o \
PrintMessage.o \
Preparizable.o \
Command.o \
Predicate.o \
CommandAndPredicate.o \
NumberOfTimesPredicate.o \
WheelController.o \
MotorCountPredicate.o \
StartButtonPredicate.o \
CommandExecutor.o \
RotateRobot.o \
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
SteeringRobot.o \
MotorRotateAnglePredicate.o \
Stopper.o \
ObstacleDetector.o \
ObstacleDetectRunner.o \
SwingSonarObstacleDetector.o \
FinishConfirmable.o \
FinishedCommandPredicate.o \
DistancePredicate.o \
CurvatureWalker.o \
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
	$(mkfile_path)robot

INCLUDES += \
	-I$(mkfile_path)command \
	-I$(mkfile_path)handler \
	-I$(mkfile_path)predicate \
	-I$(mkfile_path)robot

#COPTS += -fno-use-cxa-atexit

COPTS += -DMAKE_EV3
