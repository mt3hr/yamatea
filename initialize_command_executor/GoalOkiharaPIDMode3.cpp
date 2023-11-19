#include "Setting.h"
#ifdef GoalOkiharaPIDMode3

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
  int sceneCarrotMotorCountPredicateArg = 750;      // スタートから最初のカーブ終わるまで
  int sceneBananaMotorCountPredicateArg = 1100;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int scenePeachMotorCountPredicateArg = 1640;      // バナナとオレンジの間の小さいカーブ
  int sceneOrangeMotorCountPredicateArg = 2400;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2490; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2700;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 6070; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  int sceneBokChoyMotorCountPredicateArg = 6350;    // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 6470;     // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneAsparagusMotorCountPredicateArg = 7050;  // ドリアン終了後の１つ目の直線
  int sceneRadishMotorCountPredicateArg = 7340;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
  int sceneMelonMotorCountPredicateArg = 7890;      // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneLemonMotorCountPredicateArg = 8640;
  int sceneCucumberMotorCountPredicateArg = 10505;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 11240; // ゴールまで。いちご好き。ライントレースする。
  int sceneCabbageMotorCountpredicateArg = 11940;    // ゴールまで。

  float distanceTemp = 0;
  int carrotDistance = (sceneCarrotMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += carrotDistance;
  int bananaDistance = (sceneBananaMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += bananaDistance;
  int peachDistance = (scenePeachMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += peachDistance;
  int orangeDistance = (sceneOrangeMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += orangeDistance;
  int starFruitsDistance = (sceneStarFruitsMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += starFruitsDistance;
  int cherryDistance = (sceneCherryMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cherryDistance;
  int waterMelonDistance = (sceneWaterMelonMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += waterMelonDistance;
  int bokChoyDistance = (sceneBokChoyMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += bokChoyDistance;
  int dorianDistance = (sceneDorianMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += dorianDistance;
  int asparagusDistance = (sceneAsparagusMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += asparagusDistance;
  int radishDistance = (sceneRadishMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += radishDistance;
  int melonDistance = (sceneMelonMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += melonDistance;
  int lemonDistance = (sceneLemonMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += lemonDistance;
  int cucumberDistance = (sceneCucumberMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cucumberDistance;
  int strawberryDistance = (sceneStrawberryMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += strawberryDistance;
  int cabbageDistance = (sceneCabbageMotorCountpredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cabbageDistance;

  printf("以下出力された値をbananaDistanceとかに入れていって。");
  printf("%s: %10.f\n", GET_VARIABLE_NAME(carrotDistance), float(carrotDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(bananaDistance), float(bananaDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(peachDistance), float(peachDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(orangeDistance), float(orangeDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(starFruitsDistance), float(starFruitsDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(cherryDistance), float(cherryDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(waterMelonDistance), float(waterMelonDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(bokChoyDistance), float(bokChoyDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(dorianDistance), float(dorianDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(asparagusDistance), float(asparagusDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(radishDistance), float(radishDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(melonDistance), float(melonDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(lemonDistance), float(lemonDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(cucumberDistance), float(cucumberDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(strawberryDistance), float(strawberryDistance));
  printf("%s: %10.f\n", GET_VARIABLE_NAME(cabbageDistance), float(cabbageDistance));
  printf("以上");

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;
  float r;

  float leftPow;
  float rightPow;

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
  kp = 1.0;
  ki = 0;
  kd = 2.0;
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
  kp = 0.75;
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

  pwm = 30;
  kp = 1.0;
  ki = 0;
  kd = 3.0;
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