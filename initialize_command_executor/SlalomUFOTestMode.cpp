#include "Setting.h"
#ifdef SlalomUFOTestMode

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
  Stopper *stopper = new Stopper();

  float pwm;
  float leftPWM;
  float rightPWM;

  float distance;

  float n;
  float walkerPWM;
  float rotatePWM;
  float angle;
  int targetLeftDistance;
  int thresholdDistance;
  int targetRightDistance;

  float swingLeftAngle;
  float swingRightAngle;

  int skipFrameAfterDetectFirstObstacle;

  int numberOfTimes;

  float r;
  float theta;

  // タッチセンサ待機
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, "");

  // 旋回 -45度
  pwm = 10;
  angle = -45;
  RotateRobotUseGyroCommandAndPredicate *rotateRobotCommandAndPredicate1 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotateRobotCommandAndPredicate1->getCommand(), rotateRobotCommandAndPredicate1->getPredicate(), "rotateRobot1");

  //  直進
  leftPWM = 15;
  rightPWM = 15;
  distance = 8;
  Walker *walker1 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker1Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker1, walker1Predicate, GET_VARIABLE_NAME(walker1));

  // 旋回 45度
  pwm = 10;
  angle = 45;
  RotateRobotUseGyroCommandAndPredicate *rotateRobotCommandAndPredicate2 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotateRobotCommandAndPredicate2->getCommand(), rotateRobotCommandAndPredicate2->getPredicate(), "rotateRobot2");

  //  直進
  leftPWM = 15;
  rightPWM = 15;
  distance = 14;
  Walker *walker2 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker2Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker2, walker2Predicate, GET_VARIABLE_NAME(walker2));

  // UFO
  n = 8;
  walkerPWM = 20;
  rotatePWM = 3;
  angle = 180;
  targetLeftDistance = 25;  // これを検知した状態からはじめて
  thresholdDistance = 25;   // センサがこの長さ以上になる直前の距離と角度をLeftに保存して
  targetRightDistance = 25; // あとはSwingSonarと同じ
  skipFrameAfterDetectFirstObstacle = 0;
  UFORunner *ufoRunner1 = (new UFORunner(n, walkerPWM, rotatePWM, angle, thresholdDistance, targetRightDistance, targetLeftDistance, skipFrameAfterDetectFirstObstacle))->generateReverseCommand();
  commandExecutor->addCommand(ufoRunner1, new FinishedCommandPredicate(ufoRunner1), GET_VARIABLE_NAME(ufoRunner1));

  // 直進
  leftPWM = 15;
  rightPWM = 15;
  distance = 2.5;
  Walker *walker3 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker3Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker3, walker3Predicate, GET_VARIABLE_NAME(walker3));

  // カーブ
  pwm = 15;
  r = 13.5;
  theta = -95;
  CurvatureWalkerCommandAndPredicate *curvatureWalkerCommandAndPredicate1 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curvatureWalkerCommandAndPredicate1->getCommand(), curvatureWalkerCommandAndPredicate1->getPredicate(), "curvatureWalker1");

  /*
  // 直進
  leftPWM = 15;
  rightPWM = 15;
  distance = 4;
  Walker *walker4 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker4Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker4, walker4Predicate, GET_VARIABLE_NAME(walker4));
  */

  // ufo
  n = 8;
  walkerPWM = 20;
  rotatePWM = 3;
  swingLeftAngle = -90.0;
  swingRightAngle = 90.0;
  targetLeftDistance = 30;
  targetRightDistance = 30;
  UFORunner *ufoRunner2 = new UFORunner(n, walkerPWM, rotatePWM, swingLeftAngle, swingRightAngle, targetLeftDistance, targetRightDistance);
  commandExecutor->addCommand(ufoRunner2, new FinishedCommandPredicate(ufoRunner2), GET_VARIABLE_NAME(ufoRunner2));

  // 直進
  leftPWM = 15;
  rightPWM = 15;
  distance = 30;
  Walker *walker5 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));

  // 旋回 -90度
  pwm = 10;
  angle = -90;
  RotateRobotUseGyroCommandAndPredicate *rotateRobotCommandAndPredicate3 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotateRobotCommandAndPredicate3->getCommand(), rotateRobotCommandAndPredicate3->getPredicate(), "rotateRobot3");

  // ufo
  n = 8;
  walkerPWM = 20;
  rotatePWM = 3;
  swingLeftAngle = -90.0;
  swingRightAngle = 90.0;
  targetLeftDistance = 10;
  targetRightDistance = 20;
  UFORunner *ufoRunner3 = new UFORunner(n, walkerPWM, rotatePWM, swingLeftAngle, swingRightAngle, targetLeftDistance, targetRightDistance);
  commandExecutor->addCommand(ufoRunner3, new FinishedCommandPredicate(ufoRunner3), GET_VARIABLE_NAME(ufoRunner3));

  // 停止コマンドの初期化とCommandExecutorへの追加
  numberOfTimes = 1;
  Predicate *stopperPredicate = new NumberOfTimesPredicate(numberOfTimes);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif