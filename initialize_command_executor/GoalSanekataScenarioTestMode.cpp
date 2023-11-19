#include "Setting.h"
#ifdef GoalSanekataScenarioTestMode

#include "ev3api.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "SonarSensor.h"
#include "GyroSensor.h"

#include "sstream"
#include "vector"
#include "string"
#include "math.h"
#include "exception"

#include "PrintMessage.h"
#include "Command.h"
#include "CommandExecutor.h"
#include "Predicate.h"
#include "ANDPredicate.h"
#include "ORPredicate.h"
#include "PIDTracer.h"
#include "PIDTracerV2.h"
#include "PIDTracerV2.h"
#include "PIDStraightWalker.h"
#include "Walker.h"
#include "ArmController.h"
#include "ColorPredicate.h"
#include "DistanceReader.h"
#include "StartButtonPredicate.h"
#include "MotorCountPredicate.h"
#include "CommandAndPredicate.h"
#include "MotorRotateAnglePredicate.h"
#include "NumberOfTimesPredicate.h"
#include "Stopper.h"
#include "RGBRawReader.h"
#include "WheelDistancePredicate.h"
#include "RotateRobotCommandAndPredicate.h"
#include "FinishedCommandPredicate.h"
#include "CurvatureWalkerCommandAndPredicate.h"
#include "SwingSonarObstacleDetector.h"
#include "UFORunner.h"
#include "RobotAPI.h"
#include "GyroRotateAnglePredicate.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "ColorReaderUseRaw.h"
#include "CardColorReaderUseRaw.h"
#include "DebugUtil.h"
#include "Bluetooth.h"
#include "ColorPIDTracer.h"
#include "ColorPIDTracerV2.h"
#include "ColorPIDTracerV2.h"
#include "PIDTargetColorBrightnessCalibrator.h"
#include "TailController.h"
#include "MusicalScore.h"
#include "StartCyc.h"
#include "RawColorPredicate.h"
#include "ColorIDReader.h"
#include "FacingAngleAbs.h"
#include "FacingAngle.h"
#include "PIDFacingAngleAbs.h"
#include "PIDFacingAngle.h"
#include "ResetGyroSensor.h"
#include "ResetMeasAngle.h"
#include "Hedgehog.h"
#include "HedgehogUsePID.h"
#include "BatteryPredicate.h"
#include "RotateRobotCommandAndPredicateV2.h"
#include "FacingRobotUseWheelPredicate.h"
#include "DealingWithGarage.h"
#include "TimerPredicate.h"
#include "BrightnessReader.h"
#include "ResetArmAngle.h"
#include "ReleaseWheel.h"
#include "SonarDistancePredicate.h"
#include "AngleAbsPredicate.h"
#include "SetPWMCoefficient.h"
#include "ResetPWMCoefficient.h"
#include "WalkerR.h"
#include "PIDLimTracer.h"
#include "BatteryEaterSilent.h"

using namespace std;
using namespace ev3api;

void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float coefficientDistance = 0.8;  // 0.75;
  float coefficientAngle = 1;       // 0.98;
  float coefficientCurvePWM = 0.85; // 0.89;
#ifndef SimulatorMode
  commandExecutor->addCommand(new ArmController(-50), new NumberOfTimesPredicate(10), "arm down");
#endif

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "stopper");
  commandExecutor->addCommand(new ResetGyroSensor(), new NumberOfTimesPredicate(1), "reset gyro sensor");
  commandExecutor->addCommand(new ResetMeasAngle(), new NumberOfTimesPredicate(1), "reset wheel angle");

  CWCAPMode curveMode = CWCMP_Gyro;
  FacingAngleMode facingAngleMode = FA_Gyro;
  // AngleAbsPredicateMode angleAbsPredicateMode = AAPM_WheelCount;
#ifdef SimulatorMode
  float basePWM = 30;
  float straightPWM = 30;
  float lowStraightPWM = 30;
#else
  float basePWM = 40;
  float straightPWM = 50;
  float lowStraightPWM = 25;
#endif
  float radius;
  float theta;
  float angle;
  float distance;
#ifdef SimulatorMode
  float kp = 0.7;
  float ki = 0.2;
  float kd = 0.7;
  float dt = 1;
  float pwm = 80;
  float straightKp = 1;
  float straightKi = 0;
  float straightKd = 1;
  float straightDt = 1;
  float faKp = 0.7;
  float faKi = 0.01;
  float faKd = 0.7;
  float faDt = 1;
#else
  float kp = 0.45;
  float ki = 0;
  float kd = 1;
  float dt = 1;
  float pwm = 80;
  float straightKp = 0.05;
  float straightKi = 0;
  float straightKd = 0.05;
  float straightDt = 1;
  float faKp = 0.7;
  float faKi = 0.01;
  float faKd = 0.7;
  float faDt = 1;
#endif
  pwm = 10;
  ColorPIDTracer *colorPIDTracerRight = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  ColorPIDTracer *colorPIDTracerLeft = new ColorPIDTracer(LEFT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  calibrator->addColorPIDTracer(colorPIDTracerLeft);
  calibrator->addColorPIDTracer(colorPIDTracerRight);

  pwm = straightPWM;
  distance = 55 * coefficientDistance;
  PIDStraightWalker *walker1ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker1ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker1ys, walker1ysPredicate, GET_VARIABLE_NAME(walker1ys));

  angle = 90 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle1ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle1ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle1ys)), GET_VARIABLE_NAME(facingAngle1ys));

  pwm = straightPWM;
  distance = 85 * coefficientDistance;
  PIDStraightWalker *walker2ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker2ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker2ys, walker2ysPredicate, GET_VARIABLE_NAME(walker2ys));

  angle = 180 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle2ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle2ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle2ys)), GET_VARIABLE_NAME(facingAngle2ys));

  pwm = straightPWM;
  distance = 15 * coefficientDistance;
  PIDStraightWalker *walker3ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker3ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker3ys, walker3ysPredicate, GET_VARIABLE_NAME(walker3ys));

  pwm = basePWM * coefficientCurvePWM;
  radius = 28;
  theta = -170;
  CommandAndPredicate *curve3ys = new CurvatureWalkerCommandAndPredicate(curveMode, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve3ys->getCommand(), curve3ys->getPredicate(), GET_VARIABLE_NAME(curve3ys));

  angle = 0 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle3ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle3ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle3ys)), GET_VARIABLE_NAME(facingAngle3ys));

  pwm = straightPWM;
  distance = 10 * coefficientDistance;
  PIDStraightWalker *walker4ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker4ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker4ys, walker4ysPredicate, GET_VARIABLE_NAME(walker4ys));

  pwm = basePWM * coefficientCurvePWM;
  radius = 45;
  theta = 320;
  CommandAndPredicate *curve4ys = new CurvatureWalkerCommandAndPredicate(curveMode, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve4ys->getCommand(), curve4ys->getPredicate(), GET_VARIABLE_NAME(curve4ys));

  angle = 320 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle4ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle4ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle4ys)), GET_VARIABLE_NAME(facingAngle4ys));

  pwm = straightPWM;
  distance = 130 * coefficientDistance;
  PIDStraightWalker *walker5ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker5ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5ys, walker5ysPredicate, GET_VARIABLE_NAME(walker5ys));

  angle = 270 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle5ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle5ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle5ys)), GET_VARIABLE_NAME(facingAngle5ys));

  pwm = straightPWM;
  distance = 70 * coefficientDistance;
  PIDStraightWalker *walker6ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker6ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker6ys, walker6ysPredicate, GET_VARIABLE_NAME(walker6ys));

  angle = 360 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle6ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle6ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle6ys)), GET_VARIABLE_NAME(facingAngle6ys));

  pwm = straightPWM;
  distance = 50 * coefficientDistance;
  PIDStraightWalker *walker7ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker7ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker7ys, walker7ysPredicate, GET_VARIABLE_NAME(walker7ys));

  angle = 450 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle7ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle7ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle7ys)), GET_VARIABLE_NAME(facingAngle7ys));

  pwm = straightPWM;
  distance = 290 * coefficientDistance;
  PIDStraightWalker *walker8ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker8ysPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker8ys, walker8ysPredicate, GET_VARIABLE_NAME(walker8ys));

  angle = 345 * coefficientAngle;
  PIDFacingAngleAbs *facingAngle8ys = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
  commandExecutor->addCommand(facingAngle8ys, new ORPredicate(new TimerPredicate(1000000), new FinishedCommandPredicate(facingAngle8ys)), GET_VARIABLE_NAME(facingAngle8ys));

  pwm = lowStraightPWM;
  PIDStraightWalker *walker9ys = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
  Predicate *walker9ysPredicate = new BlackPredicate();
  commandExecutor->addCommand(walker9ys, walker9ysPredicate, GET_VARIABLE_NAME(walker9ys));

  commandExecutor->addCommand(colorPIDTracerRight, new BlueEdgePredicate(), GET_VARIABLE_NAME(colorPIDTracer));
}
#endif