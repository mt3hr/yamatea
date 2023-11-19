#include "Setting.h"
#ifdef GoalOkiharaPIDMode3Distance

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
  ResetArmAngle *resetArmAngle = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngle, new FinishedCommandPredicate(resetArmAngle), GET_VARIABLE_NAME(resetArmAngle));

  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  // スタート後メッセージ出力コマンドの初期化とCommandExecutorへの追加
  vector<string> messageLines;
  messageLines.push_back("Started!!");
  messageLines.push_back("GOGOGO!!");
  PrintMessage *printMessage = new PrintMessage(messageLines, true);
  Predicate *printMessagePredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(printMessage, printMessagePredicate, GET_VARIABLE_NAME(printMessage));

  // ガレージカードの色取得用ColorReader
  // ColorReader *colorReader = new ColorReader();
  // colorid_t *garageCardColorPtr = colorReader->getColorPtr();

  float carrotDistance = 68;
  float bananaDistance = 31;
  float peachDistance = 49;
  float orangeDistance = 69;
  float starFruitsDistance = 8;
  float cherryDistance = 20;
  float waterMelonDistance = 305;
  float bokChoyDistance = 26;
  float dorianDistance = 11;
  float asparagusDistance = 52;
  float radishDistance = 27;
  float melonDistance = 50;
  float lemonDistance = 68;
  float cucumberDistance = 169;
  float strawberryDistance = 67;
  float cabbageDistance = 63;

#ifdef Right
  orangeDistance += 1.5;
  waterMelonDistance += 5;
#endif

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;
  float r;

  float leftPow;
  float rightPow;
  // CarrotPIDTracerの初期化とCommandExecutorへの追加
  // 下記コメントアウト箇所アンパイ

  pwm = 25;
  kp = 0.9;
  ki = 0.06;
  kd = 1.4;
  dt = 1;
  r = 0;

  /*
  pwm = 30;
  kp = 1.0;
  ki = 0;
  kd = 3.0;
  dt = 1;
  r = 0;
  */

  PIDTracerV2 *carrotPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateCarrot = new WheelDistancePredicate(carrotDistance, robotAPI);
  commandExecutor->addCommand(carrotPIDTracer, predicateCarrot, GET_VARIABLE_NAME(carrotPIDTracer));
  calibrator->addPIDTracer(carrotPIDTracer);

  // BananaPIDTracerの初期化とCommandExecutorへの追加

  pwm = 30;
  kp = 0.45;
  ki = 0.01;
  kd = 1.5;
  dt = 1;
  r = 0;

  PIDTracerV2 *bananaPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
  calibrator->addPIDTracer(bananaPIDTracer);

  // PeachPIDTracerの初期化とCommandExecutorへの追加
  // アンパイ

  pwm = 25;
  kp = 0.8;
  ki = 0.05;
  kd = 1.8;
  dt = 1;
  r = 0;

  /*
  pwm = 30;
  kp = 1.2;
  ki = 0;
  kd = 3.6;
  dt = 1;
  r = 0;
  */

  PIDTracerV2 *peachPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicatePeach = new WheelDistancePredicate(peachDistance, robotAPI);
  calibrator->addPIDTracer(peachPIDTracer);
  commandExecutor->addCommand(peachPIDTracer, predicatePeach, GET_VARIABLE_NAME(peachPIDTracer));

  // OrangePIDTracerの初期化とCommandExecutorへの追加

  /*
  pwm = 30;
  kp = 0.7;
  ki = 0.01;
  kd = 1.8;
  dt = 1;
  r = 0;
  */

  pwm = 40;
  kp = 1.15; // 1.0;
  ki = 0;
  kd = 2.5; // 2.0;
  dt = 1;
  r = 0;

  PIDTracerV2 *orangePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
  calibrator->addPIDTracer(orangePIDTracer);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

  // StarFruitsWalkerの初期化とCommandExecutorへの追加

  leftPow = 9;
  rightPow = 15;

  Walker *starFruitsWalker = new Walker(leftPow, rightPow);
  Predicate *predicateStarFruits = new WheelDistancePredicate(starFruitsDistance, robotAPI);
  commandExecutor->addCommand(starFruitsWalker, predicateStarFruits, GET_VARIABLE_NAME(starFruitsWalker));

  // CherryPIDTracerの初期化とCommandExecutorへの追加

  pwm = 25;
  kp = 0.8;
  ki = 0;
  kd = 1.8;
  dt = 1;
  r = 0;

  PIDTracerV2 *cherryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateCherry = new WheelDistancePredicate(cherryDistance, robotAPI);
  calibrator->addPIDTracer(cherryPIDTracer);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry, GET_VARIABLE_NAME(cherryPIDTracer));

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加

  pwm = 40;
  kp = 0.65; // 0.75;
  ki = 0;
  kd = 2.5;
  dt = 1;
  r = 0;

  /* 下よりいい数値
  pwm = 40;
  kp = 0.8;
  ki = 0;
  kd = 2.5;
  dt = 1;
  r = 0;
  */

  // 下記はアンパイ
  /*
  pwm = 30;
  kp = 0.7;
  ki = 0;
  kd = 1.5;
  dt = 1;
  r = 0;*/

  PIDTracerV2 *waterMelonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
  calibrator->addPIDTracer(waterMelonPIDTracer);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

  // BokChoyWalkerの初期化とCommandExecutorへの追加

  leftPow = 50;
  rightPow = 50;

  Walker *bokChoyWalker = new Walker(leftPow, rightPow);
  Predicate *predicateBokChoy = new WheelDistancePredicate(bokChoyDistance, robotAPI);
  commandExecutor->addCommand(bokChoyWalker, predicateBokChoy, GET_VARIABLE_NAME(bokChoyWalker));

  // DorianPIDTracerの初期化とCommandExecutorへの追加

  pwm = 25;
  kp = 0.7;
  ki = 0;
  kd = 1.2;
  dt = 1;
  r = 0;

  PIDTracerV2 *dorianPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateDorian = new WheelDistancePredicate(dorianDistance, robotAPI);
  calibrator->addPIDTracer(dorianPIDTracer);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian, GET_VARIABLE_NAME(dorianPIDTracer));

  // AsparagusPIDTracerの初期化とCommandExecutorへの追加

  pwm = 30;
  kp = 0.6;
  ki = 0;
  kd = 2.0;
  dt = 1;
  r = 0;

  /*
  pwm = 25;
  kp = 0.65;
  ki = 0;
  kd = 1.2;
  dt = 1;
  r = 0;
  */

  PIDTracerV2 *asparagusPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateAsparagus = new WheelDistancePredicate(asparagusDistance, robotAPI);
  calibrator->addPIDTracer(asparagusPIDTracer);
  commandExecutor->addCommand(asparagusPIDTracer, predicateAsparagus, GET_VARIABLE_NAME(asparagusPIDTracer));

  // RadishPIDTracerの初期化とCommandExecutorへの追加

  pwm = 30;
  kp = 0.4;
  ki = 0;
  kd = 1.2;
  dt = 1;
  r = 0;

  PIDTracerV2 *radishPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateRadish = new WheelDistancePredicate(radishDistance, robotAPI);
  calibrator->addPIDTracer(radishPIDTracer);
  commandExecutor->addCommand(radishPIDTracer, predicateRadish, GET_VARIABLE_NAME(radishPIDTracer));

  // MelonPIDTracerの初期化とCommandExecutorへの追加  ここから

  pwm = 25;
  kp = 0.8;
  ki = 0.05;
  kd = 1.8;
  dt = 1;
  r = 0;

  PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
  calibrator->addPIDTracer(melonPIDTracer);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

  // LemonPIDTracerの初期化とCommandExecutorへの追加

  /*
  pwm = 30;
  kp = 1.0;
  ki = 0;
  kd = 3.0;
  dt = 1;
  r = 0;
  */
  // メロンのを使う
  pwm = 25;
  kp = 0.8;
  ki = 0.05;
  kd = 1.8;
  dt = 1;
  r = 0;

  PIDTracerV2 *lemonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateLemon = new WheelDistancePredicate(lemonDistance, robotAPI);
  calibrator->addPIDTracer(lemonPIDTracer);
  commandExecutor->addCommand(lemonPIDTracer, predicateLemon, GET_VARIABLE_NAME(lemonPIDTracer));

  // CucumberPIDTracerの初期化とCommandExecutorへの追加

  /*pwm = 60;
  kp = 0.75;
  ki = 0.01;
  kd = 1.5;
  dt = 1;
  r = 0;
  */

  pwm = 60;
  kp = 0.75;
  ki = 0;
  kd = 2.0;
  dt = 1;
  r = 0;

  PIDTracerV2 *cucumberPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
  calibrator->addPIDTracer(cucumberPIDTracer);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加

  /*pwm = 25;
  kp = 0.7;
  ki = 0;
  kd = 2.0;
  dt = 1;
  r = 0;
  */

  pwm = 30;
  kp = 0.75;
  ki = 0;
  kd = 2.5;
  dt = 1;
  r = 0;
  /*
  // メロンのを使う
  pwm = 25;
  kp = 0.8;
  ki = 0.05;
  kd = 1.8;
  dt = 1;
  r = 0;
  */

  PIDTracerV2 *strawberryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
  calibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

  // CabbagePIDTracerの初期化とCommandExecutorへの追加

  pwm = 60;
  kp = 0.75;
  ki = 0;
  kd = 2.4;
  dt = 1;
  r = 0;

  PIDTracerV2 *cabbagePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  Predicate *predicateCabbage = new WheelDistancePredicate(cabbageDistance, robotAPI);
  calibrator->addPIDTracer(cabbagePIDTracer);
  commandExecutor->addCommand(cabbagePIDTracer, predicateCabbage, GET_VARIABLE_NAME(cabbagePIDTracer));
  // Commandの定義とCommandExecutorへの追加ここまで

#if defined(SimulatorMode) | defined(DisableCalibration)
  // シミュレータはPIDTargetBrightnessをキャリブレーションしないので値を設定する必要がある
  bananaPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  orangePIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  cherryPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  waterMelonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  dorianPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  melonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  cucumberPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  strawberryPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
#endif
}
#endif