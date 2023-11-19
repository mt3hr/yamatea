mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
Util.o \
RawColorPredicate.o \
Setting.o \
Bluetooth.o \
CorrectedMotor.o \
RobotAPI.o \
PrintMessage.o \
Note.o \
Command.o \
Predicate.o \
ORPredicate.o \
ANDPredicate.o \
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
PIDStraightWalker.o \
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
CardColorReaderUseRaw.o \
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
PIDFacingAngle.o \
PIDFacingAngleAbs.o \
ResetGyroSensor.o \
ResetMeasAngle.o \
Hedgehog.o \
HedgehogUsePID.o \
BatteryPredicate.o \
RotateRobotCommandAndPredicateV2.o \
FacingRobotUseWheelPredicate.o \
DealingWithGarage.o \
TimerPredicate.o \
BrightnessReader.o \
ResetArmAngle.o \
ReleaseWheel.o \
AngleAbsPredicate.o \
SetPWMCoefficient.o \
ResetPWMCoefficient.o \
WalkerR.o \
PIDLimTracer.o \
BatteryEaterSilent.o \
BatteryEaaterMode.o \
BatteryEeaterSilentMode.o \
BlockTestMode.o \
BrightnessPIDTracerTestMode.o \
BrightnessPIDTracerV2TestMode.o \
BrightnessReaderMode.o \
ColorIDReaderMode.o \
ColorPIDTracerTestMode.o \
ColorPIDTracerV2TestMode.o \
ColorReaderUseRawTestMode.o \
CurvatureWalkerTestMode.o \
DistanceReaderMode.o \
FacingAngleTestMode.o \
FlatLineMode.o \
FreedomDiveTestMode.o \
FroggySongTestMode.o \
GarageKomichiMode1.o \
GoalKomichiScnenarioTestMode.o \
GoalOkiharaPIDMode.o \
GoalOkiharaPIDMode1.o \
GoalOkiharaPIDMode2.o \
GoalOkiharaPIDMode3.o \
GoalOkiharaPIDMode3Distance.o \
GoalOkiharaPIDMode4.o \
GoalSanekataPIDMode1.o \
GoalSanekataPIDMode2.o \
GoalSanekataPIDMode2_Stable1.o \
GoalSanekataPIDMode2_Stable2.o \
GoalSanekataPIDMode3.o \
GoalSanekataScenarioTestMode.o \
GrayPredicateTestMode.o \
OCCircleCourceMode.o \
PIDFacingAngleAbsTestMode.o \
PIDLimTracerTestMode.o \
PIDStraightWalkerTestMode.o \
RGBRawReaderMode.o \
Rotate360TestMode.o \
RotateGyroTestMode.o \
RotateTestMode.o \
ShigekiTestMode.o \
SlalomAwaitingSignalPlan1SanekataMode.o \
SlalomAwaitingSignalPlan2SanekataMode.o \
SlalomAwaitingSignalPlan3SanekataMode.o \
SlalomAwaitingSignalPlan4SanekataMode.o \
SlalomAwaitingSignalPlan5SanekataMode.o \
SlalomBlockJoinTestMode.o \
SlalomUFOTestMode.o \
StraightTestMode.o \
SwingSonarDetectorTestMode.o \
TrueCourceKomichiModeRegional.o \
TrueCourceOkiharaModeCSForKomichiRobot.o \
TrueCourceOkiharaModeCSForOkiharaRobot.o \
TrueCourceOkiharaModeCSStableValueForOkiharaRobot.o \
TrueCourceOkiharaModeCSUseRegionalValueForKomichiRobot.o \
TrueCourceOkiharaModeRegional.o \
TrueCourceSanekataMode1261b.o \
TrueCourceSanekataModeCS.o \
TrueCourceSanekataModeSimulator.o \
UFORunnerClockwiseTestMode.o \
UFORunnerSwingTestMode.o \
WalkerRTestMode.o \
WalkerTestMode.o \

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
	$(mkfile_path)etc \
	$(mkfile_path)initialize_command_executor \

INCLUDES += \
	-I$(mkfile_path)command \
	-I$(mkfile_path)command/slalom \
	-I$(mkfile_path)command/garage \
	-I$(mkfile_path)command/test \
	-I$(mkfile_path)command_and_predicate \
	-I$(mkfile_path)predicate \
	-I$(mkfile_path)robot \
	-I$(mkfile_path)music \
	-I$(mkfile_path)etc \
	-I$(mkfile_path)initialize_command_executor \

#COPTS += -fno-use-cxa-atexit

COPTS += -DMAKE_EV3

COPTS += -DMAKE_EV3

COPTS += -DMAKE_EV3
