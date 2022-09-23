mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
Util.o \
RawColorPredicate.o \
Setting.o \
Bluetooth.o \
RobotAPI.o \
PrintMessage.o \
Note.o \
Command.o \
Predicate.o \
FinishConfirmable.o \
FinishedCommandPredicate.o \
MusicalScore.o \
Stopper.o \
DebugUtil.o \
ColorPredicate.o \
CommandAndPredicate.o \
NumberOfTimesPredicate.o \
MotorCountPredicate.o \
StartButtonPredicate.o \
CommandExecutor.o \
RotateRobotCommandAndPredicate.o \
GyroRotateAnglePredicate.o \
PIDTracer.o \
PIDTracerV2.o \
ColorPIDTracer.o \
ColorPIDTracerV2.o \
PIDTargetBrightnessCalibrator.o \
PIDTargetColorBrightnessCalibrator.o \
Walker.o \
RGBRawReader.o \
DistanceReader.o \
ArmController.o \
MotorRotateAnglePredicate.o \
ObstacleDetector.o \
ObstacleDetectRunner.o \
SwingSonarObstacleDetector.o \
WheelDistancePredicate.o \
CurvatureWalkerCommandAndPredicate.o \
ClockwiseObstacleDetector.o \
UFORunner.o \
SuperSocialDistanceRunner.o \
ColorReader.o \
ColorReaderUseRaw.o \
SpiderManRunner.o \
RotateRobotDistanceAngleDetector.o \
RotateRobotUseGyroCommandAndPredicate.o \
SonarDistancePredicate.o \
TailController.o \
StartCyc.o \
ColorIDReader.o \
FacingAngleAbs.o \
FacingAngle.o \
ResetGyroSensor.o \
ResetMeasAngle.o \
Hedgehog.o \
BatteryPredicate.o \
RotateRobotCommandAndPredicateV2.o \
FacingRobotUseWheelPredicate.o \
DealingWithGarage.o \
TimerPredicate.o \
BrightnessReader.o \
ResetArmAngle.o \
ReleaseWheel.o \
AngleAbsPredicate.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += \
	$(mkfile_path)command \
	$(mkfile_path)command/slalom \
	$(mkfile_path)command/garage \
	$(mkfile_path)command/test \
	$(mkfile_path)command_and_predicate \
	$(mkfile_path)predicate \
	$(mkfile_path)robot \
	$(mkfile_path)music \

INCLUDES += \
	-I$(mkfile_path)command \
	-I$(mkfile_path)command/slalom \
	-I$(mkfile_path)command/garage \
	-I$(mkfile_path)command/test \
	-I$(mkfile_path)command_and_predicate \
	-I$(mkfile_path)predicate \
	-I$(mkfile_path)robot \
	-I$(mkfile_path)music \

#COPTS += -fno-use-cxa-atexit

COPTS += -DMAKE_EV3

COPTS += -DMAKE_EV3

COPTS += -DMAKE_EV3
