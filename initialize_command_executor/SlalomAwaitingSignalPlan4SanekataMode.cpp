#include "Setting.h"
#ifdef SlalomAwaitingSignalPlan4SanekataMode

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
  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngleAtSlalom = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngleAtSlalom, new FinishedCommandPredicate(resetArmAngleAtSlalom), GET_VARIABLE_NAME(resetArmAngleAtSlalom));

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;
  float r;

  // ↓ここから実方↓
  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "Stopper");

  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();
  // colorid_t *garageCardColorPtr = colorReader->getColorPtr();

  float slalomAngleOffset = 0;

  float coefficientPWM;
  float coefficientPWMForFacingAngle;
  float coefficientPWMForCurve;

  float radius;
  float theta;

  float angle;
  float distance;

  float leftPWM;
  float rightPWM;

  int numberOfTime;

  FacingAngleMode facingAngleMode = FA_WheelCount;

#ifdef SimulatorMode
  coefficientPWM = 2;
  coefficientPWMForCurve = 2;
  coefficientPWMForFacingAngle = 2;
#else
  coefficientPWM = 1;
  coefficientPWMForCurve = 1;
  coefficientPWMForFacingAngle = 1;
#endif

  Stopper *stopper = new Stopper();

  // スラローム進入ここから
  // コース上2つ目の青線前から開始。

#ifdef SimulatorMode
  pwm = 30 * coefficientPWM;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  r = 0;
  ColorPIDTracerV2 *pidTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, r);
  pwm = 20 * coefficientPWM;
  ColorPIDTracerV2 *lowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, r);
  pwm = 10 * coefficientPWM;
  ColorPIDTracerV2 *verryLowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, r);
#else
  pwm = 30 * coefficientPWM;
  kp = 0.155;
  ki = 0.001;
  kd = 0.572;
  dt = 1;
  r = 0;
  ColorPIDTracerV2 *pidTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, r);

  pwm = 20 * coefficientPWM;
  kp = 0.195;
  ki = 0.001;
  kd = 0.396;
  dt = 1;
  r = 0;
  ColorPIDTracerV2 *lowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, r);

  pwm = 10 * coefficientPWM;
  kp = 0.205;
  ki = 0.001;
  kd = 0.322;
  dt = 1;
  r = 0;
  ColorPIDTracerV2 *verryLowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, r);
#endif
  calibrator->addColorPIDTracer(pidTracer);
  calibrator->addColorPIDTracer(lowPWMTracer);
  calibrator->addColorPIDTracer(verryLowPWMTracer);
#ifdef SimulatorMode
  rgb_raw_t targetRGB;
  targetRGB.r = blackWhiteEdgeR;
  targetRGB.g = 60;
  targetRGB.b = 60;
  pidTracer->setTargetColor(targetRGB);
  lowPWMTracer->setTargetColor(targetRGB);
#endif

  // 色読み取りでBrightnessからRawColorに切り替える
  commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));

  // PIDトレースで青線まで進む
  Predicate *distancePredicate = new WheelDistancePredicate(40, robotAPI);
  commandExecutor->addCommand(pidTracer, distancePredicate, GET_VARIABLE_NAME(lowPWMTracer));

  // PIDトレースで青線まで進む
  Predicate *pidTracerPredicate = new BlueEdgePredicate();
  commandExecutor->addCommand(lowPWMTracer, pidTracerPredicate, GET_VARIABLE_NAME(lowPWMTracer));

  // スラローム直前までPIDトレース
  distance = 26;
  commandExecutor->addCommand(lowPWMTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // アームを下げる
  float armAngle = 15;
  pwm = -10 * coefficientPWM;
  numberOfTime = 25;
  Command *armDown = new ArmController(pwm);
  Predicate *armDownPredicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(armDown, armDownPredicate, GET_VARIABLE_NAME(armDown));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

// テールモータで角度をつける
#ifdef SimulatorMode
  pwm = 30;
#else
  pwm = 100;
#endif
  numberOfTime = 40;
  Command *tailMotorDrive = new TailController(pwm);
  Predicate *tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // スラローム位置補正。アームを下げたまま直進。
  numberOfTime = 65;
  leftPWM = 7 * coefficientPWM;
  rightPWM = 7 * coefficientPWM;
  Command *lowWalker = new Walker(leftPWM, rightPWM);
  commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
  commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

  // ジャイロセンサをリセットする
  ResetGyroSensor *resetGyroSensor = new ResetGyroSensor();
  commandExecutor->addCommand(resetGyroSensor, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetGyroSensor));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // MeasAngleをリセットする
  ResetMeasAngle *resetMeasAngle = new ResetMeasAngle();
  commandExecutor->addCommand(resetMeasAngle, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetMeasAngle));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
  leftPWM = -5 * coefficientPWM;
  rightPWM = -5 * coefficientPWM;
  distance = -4;
  Command *back = new Walker(leftPWM, rightPWM);
  Predicate *backPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(back, backPredicate, GET_VARIABLE_NAME(back));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // アームを戻す
  pwm = 10 * coefficientPWM;
  Command *armUp = new ArmController(pwm);
  Predicate *armUpPredicate = new MotorRotateAnglePredicate(armAngle, robotAPI->getArmMotor());
  commandExecutor->addCommand(armUp, armUpPredicate, GET_VARIABLE_NAME(armUp));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // テールモータを戻す

#ifdef SimulatorMode
  pwm = 30;
#else
  pwm = 100;
#endif
  numberOfTime = 20;
  tailMotorDrive = new TailController(-pwm);
  tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  distance = 29;
  leftPWM = 12 * coefficientPWM;
  rightPWM = 12 * coefficientPWM;
  Walker *walker1_y = new Walker(leftPWM, rightPWM);
  commandExecutor->addCommand(walker1_y, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(walker1_y));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // テールモータを戻す
#ifdef SimulatorMode
  pwm = 30;
#else
  pwm = 100;
#endif
  numberOfTime = 20;
  tailMotorDrive = new TailController(-pwm);
  tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // スラローム進入ここまで

  // スラローム位置補正ここから

  // ジャイロで向き調節
  // pwm = 6 * coefficientPWMForFacingAngle;
  // angle = 0;
  // FacingAngleAbs *facingAngleG = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  // commandExecutor->addCommand(facingAngleG, new FinishedCommandPredicate(facingAngleG), GET_VARIABLE_NAME(FacingAngleG));
  // commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // MeasAngleをリセットする
  // numberOfTime = 1;
  // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(numberOfTime), "releaseWheel");
  // commandExecutor->addCommand(resetMeasAngle, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetMeasAngle));
  // commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -90;
  FacingAngleAbs *facingAngleX = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngleX, new FinishedCommandPredicate(facingAngleX), GET_VARIABLE_NAME(facingAngleX));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 白を拾うまで直進
  leftPWM = 5 * coefficientPWM;
  rightPWM = 5 * coefficientPWM;
  Walker *walkerW = new Walker(leftPWM, rightPWM);

  RawColorPredicate *whitePredicate = new WhiteAtSlaromPredicate();
  commandExecutor->addCommand(walkerW, whitePredicate, GET_VARIABLE_NAME(walkerW));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // バック
  leftPWM = -6 * coefficientPWM;
  rightPWM = -6 * coefficientPWM;
  // distance = -5.2;
  distance = -4; // 大会で変更
  Walker *walkerB = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerBPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerB, walkerBPredicate, GET_VARIABLE_NAME(walkerB));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 6 * coefficientPWMForFacingAngle;
  angle = 0;
  FacingAngleAbs *facingAngleC = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngleC, new FinishedCommandPredicate(facingAngleC), GET_VARIABLE_NAME(facingAngleC));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  /*
  leftPWM = 8;
  rightPWM = 8;
  Walker *walkerC2 = new Walker(leftPWM, rightPWM);
  Predicate *walkerC2Predicate = new WheelDistancePredicate(6, robotAPI);
  commandExecutor->addCommand(walkerC2, walkerC2Predicate, GET_VARIABLE_NAME(walkerC2));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  pwm = 6 * coefficientPWMForFacingAngle;
  angle = 0;
  FacingAngleAbs *facingAngleC2 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngleC2, new FinishedCommandPredicate(facingAngleC2), GET_VARIABLE_NAME(facingAngleC2));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  leftPWM = 8;
  rightPWM = 8;
  Walker *walkerC3 = new Walker(leftPWM, rightPWM);
  Predicate *walkerC3Predicate = new WheelDistancePredicate(6, robotAPI);
  commandExecutor->addCommand(walkerC3, walkerC3Predicate, GET_VARIABLE_NAME(walkerC3));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  pwm = 6 * coefficientPWMForFacingAngle;
  angle = 0;
  FacingAngleAbs *facingAngleC3 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngleC3, new FinishedCommandPredicate(facingAngleC3), GET_VARIABLE_NAME(facingAngleC3));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  */

  // スラローム位置補正ここまで

  // 指示待ち走行ここから

  // 向き調節
  pwm = 8 * coefficientPWMForFacingAngle;
  angle = 0;
  FacingAngleAbs *facingAngle1 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle1, new FinishedCommandPredicate(facingAngle1), GET_VARIABLE_NAME(facingAngle1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 位置調節
  pwm = 8 * coefficientPWM;
  distance = 10;
  Hedgehog *headgehogA = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehogA, new FinishedCommandPredicate(headgehogA), GET_VARIABLE_NAME(headgehogA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 10 * coefficientPWMForCurve;
  radius = 17.5;
  theta = 50;
  CurvatureWalkerCommandAndPredicate *curve1 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve1->getCommand(), curve1->getPredicate(), GET_VARIABLE_NAME(curve1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  /*
  // 直進
  leftPWM = 7 * coefficientPWM;
  rightPWM = 7 * coefficientPWM;
  distance = 3.9;
  Walker *walkerA = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerAPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerA, walkerAPredicate, GET_VARIABLE_NAME(walkerA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  */

  // カーブ
  pwm = 7 * coefficientPWMForCurve;
  radius = 15.5;
  theta = -45;
  CurvatureWalkerCommandAndPredicate *curve2 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve2->getCommand(), curve2->getPredicate(), GET_VARIABLE_NAME(curve2));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = 0;
  FacingAngleAbs *facingAngle3 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle3, new FinishedCommandPredicate(facingAngle3), GET_VARIABLE_NAME(facingAngle3));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 位置調節
  pwm = 7 * coefficientPWM;
  distance = 9;
  Hedgehog *headgehog1 = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehog1, new FinishedCommandPredicate(headgehog1), GET_VARIABLE_NAME(headgehog1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 10 * coefficientPWMForCurve;
  radius = 15;
  theta = -35;
  CurvatureWalkerCommandAndPredicate *curve3 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve3->getCommand(), curve3->getPredicate(), GET_VARIABLE_NAME(curve3));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 8;
  Walker *walkerD = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerDPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerD, walkerDPredicate, GET_VARIABLE_NAME(walkerD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -40;
  FacingAngleAbs *facingAngle4 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle4, new FinishedCommandPredicate(facingAngle4), GET_VARIABLE_NAME(facingAngle4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 7 * coefficientPWMForCurve;
  radius = 8;
  theta = 24;
  CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // バック
  leftPWM = 7 * coefficientPWM;
  rightPWM = 7 * coefficientPWM;
  distance = 1.5;
  Walker *walkerZ = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerZPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerZ, walkerZPredicate, GET_VARIABLE_NAME(walkerZ));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 13;
  theta = 55;
  CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  /*
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 3;
  Walker *walker5 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  */

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 22.5;
  theta = -30;
  CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  FacingAngleAbs *facingAngle5 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset);
  commandExecutor->addCommand(facingAngle5, new FinishedCommandPredicate(facingAngle5), GET_VARIABLE_NAME(facingAngle5));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進位置調節
  int diff = 0;
  pwm = 10 * coefficientPWM;
  distance = 4 + diff;
  Hedgehog *headgehog2 = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehog2, new FinishedCommandPredicate(headgehog2), GET_VARIABLE_NAME(headgehog2));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = 22.5;
  FacingAngleAbs *facingAngleCo1 = new FacingAngleAbs(facingAngleMode, pwm, angle + slalomAngleOffset);
  commandExecutor->addCommand(facingAngleCo1, new FinishedCommandPredicate(facingAngleCo1), GET_VARIABLE_NAME(facingAngleCo1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 色取得
  commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  FacingAngleAbs *facingAngleCo2 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset);
  commandExecutor->addCommand(facingAngleCo2, new FinishedCommandPredicate(facingAngleCo2), GET_VARIABLE_NAME(facingAngleCo2));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進位置調節
  pwm = 10 * coefficientPWM;
  distance = 12 + diff;
  Hedgehog *headgehogAA = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehogAA, new FinishedCommandPredicate(headgehogAA), GET_VARIABLE_NAME(headgehogAA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

#ifdef SlalomPattern1

  // カーブ
  pwm = 10 * coefficientPWMForCurve;
  radius = 10;
  theta = -82.5;
  CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 90度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -90;
  FacingAngleAbs *facingAngleY = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngleY, new FinishedCommandPredicate(facingAngleY), GET_VARIABLE_NAME(facingAngleY));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 位置調節
  pwm = 6 * coefficientPWM;
  distance = 3;
  Hedgehog *headgehog3 = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehog3, new FinishedCommandPredicate(headgehog3), GET_VARIABLE_NAME(headgehog3));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 6 * coefficientPWM;
  rightPWM = 6 * coefficientPWM;
  distance = 1.5;
  Walker *walker7 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 150度左を向く
  pwm = 5 * coefficientPWMForFacingAngle;
  angle = -150;
  FacingAngleAbs *facingAngle8 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle8, new FinishedCommandPredicate(facingAngle8), GET_VARIABLE_NAME(facingAngle8));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 10 * coefficientPWMForCurve;
  radius = 33;
  theta = 70;
  CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 30度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -30;
  FacingAngleAbs *facingAngle9 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle9, new FinishedCommandPredicate(facingAngle9), GET_VARIABLE_NAME(facingAngle9));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 黒線まで直進する
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  Walker *walkerO = new Walker(leftPWM, rightPWM);
  Predicate *blackPredicate = new BlackPredicate();
  commandExecutor->addCommand(walkerO, blackPredicate, GET_VARIABLE_NAME(walkerO));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 青線までPIDトレースする
  RawColorPredicate *blueEdgePredicate = new BlueEdgePredicate();
  commandExecutor->addCommand(verryLowPWMTracer, blueEdgePredicate, GET_VARIABLE_NAME(lowPWMTracer));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

#endif
#ifdef SlalomPattern2

  // カーブ
  pwm = 10 * coefficientPWMForCurve;
  radius = 10;
  theta = -40;
  CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 120度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -130;
  FacingAngleAbs *facingAngle7 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle7, new FinishedCommandPredicate(facingAngle7), GET_VARIABLE_NAME(facingAngle7));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 6.5;
  Walker *walker7 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 10 * coefficientPWMForCurve;
  radius = 28;
  theta = 40;
  CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 90度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -90;
  FacingAngleAbs *facingAngle8 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle8, new FinishedCommandPredicate(facingAngle8), GET_VARIABLE_NAME(facingAngle8));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  /*
  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 2;
  Walker *walker8 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker8Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker8, walker8Predicate, GET_VARIABLE_NAME(walker8));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  */

  // カーブ
  pwm = 10 * coefficientPWMForCurve;
  radius = 28;
  theta = 40;
  CurvatureWalkerCommandAndPredicate *curve7 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve7->getCommand(), curve7->getPredicate(), GET_VARIABLE_NAME(curve7));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 15;
  Walker *walker9 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker9Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker9, walker9Predicate, GET_VARIABLE_NAME(walker9));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  /*
  // 90度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -90;
  FacingAngleAbs *facingAngle9 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle9, new FinishedCommandPredicate(facingAngle9), GET_VARIABLE_NAME(facingAngle9));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  */

  /*
  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 1;
  Walker *walker10y = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker10yPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker10y, walker10yPredicate, GET_VARIABLE_NAME(walker10y));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  */

  // 110度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -120;
  FacingAngleAbs *facingAngle10 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle10, new FinishedCommandPredicate(facingAngle10), GET_VARIABLE_NAME(facingAngle10));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 黒線まで直進する
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  Walker *walkerO = new Walker(leftPWM, rightPWM);
  Predicate *blackPredicate = new BlackPredicate();
  commandExecutor->addCommand(walkerO, blackPredicate, GET_VARIABLE_NAME(walkerO));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 青線までPIDトレースする
  RawColorPredicate *blueEdgePredicate = new BlueEdgePredicate();
  commandExecutor->addCommand(lowPWMTracer, blueEdgePredicate, GET_VARIABLE_NAME(lowPWMTracer));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

#endif
  // ↑ここまで実方↑
}
#endif