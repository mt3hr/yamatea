mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
RobotAPI.o \
PrintMessage.o \
Setting.o \
DebugUtil.o \
Command.o \
Predicate.o \
CommandAndPredicate.o \
NumberOfTimesPredicate.o \
MotorCountPredicate.o \
StartButtonPredicate.o \
CommandExecutor.o \
RotateRobotCommandAndPredicate.o \
GyroRotateAnglePredicate.o \
SuitableForRightCourse.o \
PIDTargetBrightnessCalibrator.o \
PIDTracer.o \
Walker.o \
RGBRawReader.o \
DistanceReader.o \
ArmController.o \
MotorRotateAnglePredicate.o \
Stopper.o \
ObstacleDetector.o \
ObstacleDetectRunner.o \
SwingSonarObstacleDetector.o \
FinishConfirmable.o \
FinishedCommandPredicate.o \
DistancePredicate.o \
CurvatureWalkerCommandAndPredicate.o \
UFORunner.o \
AwaitingSignalRunner.o \
StraightBetweenRunner.o \
SuperSocialDistanceRunner.o \
RotateRobotDistanceAngleDetector.o \
RotateRobotUseGyroCommandAndPredicate.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += \
	$(mkfile_path)command \
	$(mkfile_path)command/slalom \
	$(mkfile_path)command/test \
	$(mkfile_path)command_and_predicate \
	$(mkfile_path)predicate \
	$(mkfile_path)robot

INCLUDES += \
	-I$(mkfile_path)command \
	-I$(mkfile_path)command/slalom \
	-I$(mkfile_path)command/test \
	-I$(mkfile_path)command_and_predicate \
	-I$(mkfile_path)predicate \
	-I$(mkfile_path)robot

#COPTS += -fno-use-cxa-atexit

COPTS += -DMAKE_EV3

COPTS += -DMAKE_EV3
