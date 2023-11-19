#include "Setting.h"
#ifdef SlalomAwaitingSignalPlan3SanekataMode

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
  // ガレージカードの色取得用ColorReader
  // ColorReader *colorReader = new ColorReader();

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

  float r;
  float theta;

  float angle;

  float leftPWM;
  float rightPWM;

  int numberOfTime;

  Stopper *stopper = new Stopper();
  // Predicate *stopperPredicate = new NumberOfTimesPredicate(1);

  // Calibrator
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  // スラローム進入ここから
  // コース上2つ目の青線前から開始。
  kp = 0.2;
  ki = 0.1;
  kd = 0.2;
  dt = 1;

  pwm = 15;
  ColorPIDTracer *pidTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 8;
  ColorPIDTracer *lowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  calibrator->addColorPIDTracer(pidTracer);
  calibrator->addColorPIDTracer(lowPWMTracer);

#ifdef SimulatorMode
  rgb_raw_t targetRGB;
  targetRGB.r = 60;
  targetRGB.g = 60;
  targetRGB.b = 60;
  pidTracer->setTargetColor(targetRGB);
  lowPWMTracer->setTargetColor(targetRGB);
#endif
  // PIDトレースから入る
  commandExecutor->addCommand(lowPWMTracer, new WheelDistancePredicate(5, robotAPI), GET_VARIABLE_NAME(pidTracer));

  // PIDトレースで青線まで進む
  Predicate *pidTracerPredicate = new BlueEdgePredicate();
  commandExecutor->addCommand(lowPWMTracer, pidTracerPredicate, GET_VARIABLE_NAME(pidTracer));

  // スラローム直前までPIDトレース
  float distance = 30;
  commandExecutor->addCommand(pidTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // アームを下げる
  float armAngle = 15;
  pwm = -10;
  numberOfTime = 25;
  Command *armDown = new ArmController(pwm);
  Predicate *armDownPredicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(armDown, armDownPredicate, GET_VARIABLE_NAME(armDown));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // テールモータで角度をつける
  pwm = 50;
  numberOfTime = 25;
  Command *tailMotorDrive = new TailController(pwm);
  Predicate *tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

// スラローム位置補正。アームを下げたまま直進。
#ifdef SimulatorMode
  numberOfTime = 50;
#else
  numberOfTime = 50;
#endif
  leftPWM = 10;
  rightPWM = 10;
  Command *lowWalker = new Walker(leftPWM, rightPWM);
  commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
  Command *back = new Walker(-5, -5);
  distance = -3;
  Predicate *backPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(back, backPredicate, GET_VARIABLE_NAME(back));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // アームを戻す
  pwm = 10;
  Command *armUp = new ArmController(pwm);
  Predicate *armUpPredicate = new MotorRotateAnglePredicate(armAngle, robotAPI->getArmMotor());
  commandExecutor->addCommand(armUp, armUpPredicate, GET_VARIABLE_NAME(armUp));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // テールモータを戻す
  pwm = 30;
  numberOfTime = 25;
  tailMotorDrive = new TailController(-pwm);
  tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  distance = 27;
  Walker *walker1 = new Walker(20, 20);
  commandExecutor->addCommand(walker1, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(walker1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // スラローム進入ここまで

  // 指示待ち走行ここから

  // 45度左旋回
  pwm = 10;
  angle = -45;
  CommandAndPredicate *rotate1 = new RotateRobotCommandAndPredicateV2(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate1->getCommand(), rotate1->getPredicate(), GET_VARIABLE_NAME(rotate1));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 14;
  Walker *walker2 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker2Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker2, walker2Predicate, GET_VARIABLE_NAME(walker2));

  // 45度右旋回
  pwm = 10;
  angle = 45;
  CommandAndPredicate *rotate2 = new RotateRobotCommandAndPredicateV2(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate2->getCommand(), rotate2->getPredicate(), GET_VARIABLE_NAME(rotate2));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 16;
  Walker *walker3 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker3Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker3, walker3Predicate, GET_VARIABLE_NAME(walker3));

  // 45度右旋回
  pwm = 10;
  angle = 45;
  CommandAndPredicate *rotate3 = new RotateRobotCommandAndPredicateV2(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate3->getCommand(), rotate3->getPredicate(), GET_VARIABLE_NAME(rotate3));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 12;
  Walker *walker4 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker4Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker4, walker4Predicate, GET_VARIABLE_NAME(walker4));

  // 45度左旋回
  pwm = 10;
  angle = -45;
  CommandAndPredicate *rotate4 = new RotateRobotCommandAndPredicateV2(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate4->getCommand(), rotate4->getPredicate(), GET_VARIABLE_NAME(rotate4));

  // カーブ
  pwm = 10;
  r = 18;
  theta = -80;
  CommandAndPredicate *curve1 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve1->getCommand(), curve1->getPredicate(), GET_VARIABLE_NAME(curve1));

  // 90度右旋回
  pwm = 10;
  angle = 90;
  CommandAndPredicate *rotateA = new RotateRobotCommandAndPredicateV2(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotateA->getCommand(), rotateA->getPredicate(), GET_VARIABLE_NAME(rotateA));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 10;
  Walker *walkerA = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerAPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerA, walkerAPredicate, GET_VARIABLE_NAME(walkerA));

  // 45度右旋回
  pwm = 10;
  angle = 45;
  CommandAndPredicate *rotate6 = new RotateRobotCommandAndPredicateV2(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate6->getCommand(), rotate6->getPredicate(), GET_VARIABLE_NAME(rotate6));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 20;
  Walker *walker7 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));

  // 20度左旋回
  pwm = 10;
  angle = -20;
  CommandAndPredicate *rotate7 = new RotateRobotCommandAndPredicateV2(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate7->getCommand(), rotate7->getPredicate(), GET_VARIABLE_NAME(rotate7));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 16;
  Walker *walker8 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker8Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker8, walker8Predicate, GET_VARIABLE_NAME(walker8));
}
#endif