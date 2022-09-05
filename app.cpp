// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// 実方
#include "app.h"
#include "Setting.h"

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

#include "PrintMessage.h"
#include "Command.h"
#include "CommandExecutor.h"
#include "Predicate.h"
#include "PIDTracer.h"
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
#include "ColorReader.h"
#include "DebugUtil.h"
#include "Bluetooth.h"
#include "ColorPIDTracer.h"
#include "PIDTargetColorBrightnessCalibrator.h"
#include "TailController.h"
#include "MusicalScore.h"
#include "StartCyc.h"
#include "RawColorPredicate.h"

using namespace std;
using namespace ev3api;

CommandExecutor *commandExecutor;
RobotAPI *robotAPI;

// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#if defined(LeftCourceMode) | defined(RightCourceMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  float bananaDistance = 109;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  float orangeDistance = 113;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  float starFruitsDistance = 9;   // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  float cherryDistance = 18;      // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  float waterMelonDistance = 300; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  float bokChoyDistance = 35;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  float dorianDistance = 25;      // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  float melonDistance = 209;      // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  float cucumberDistance = 140;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  float strawberryDistance = 140; // ゴールまで。いちご好き。ライントレースする。

  /*
  // TODO ここから消そう
  int sceneBananaMotorCountPredicateArg = 1200;      // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int sceneOrangeMotorCountPredicateArg = 2450;      // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2550;  // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2750;      // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 5150;  // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  int sceneBokChoyMotorCountPredicateArg = 5400;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 5700;      // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneMelonMotorCountPredicateArg = 8000;       // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneCucumberMotorCountPredicateArg = 9700;    // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 11200; // ゴールまで。いちご好き。ライントレースする。

  float distanceTemp = 0;
  bananaDistance = (sceneBananaMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += bananaDistance;
  orangeDistance = (sceneOrangeMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += orangeDistance;
  starFruitsDistance = (sceneStarFruitsMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += starFruitsDistance;
  cherryDistance = (sceneCherryMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cherryDistance;
  waterMelonDistance = (sceneWaterMelonMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += waterMelonDistance;
  bokChoyDistance = (sceneBokChoyMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += bokChoyDistance;
  dorianDistance = (sceneDorianMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += dorianDistance;
  melonDistance = (sceneMelonMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += melonDistance;
  cucumberDistance = (sceneCucumberMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cucumberDistance;
  strawberryDistance = (sceneStrawberryMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += strawberryDistance;

  printf("以下出力された値をbananaDistanceとかに入れていって。");
  printf("%sDistance: %10.f\n", "Banana", bananaDistance);
  printf("%sDistance: %10.f\n", "Orange", orangeDistance);
  printf("%sDistance: %10.f\n", "StarFruits", starFruitsDistance);
  printf("%sDistance: %10.f\n", "Cherry", cherryDistance);
  printf("%sDistance: %10.f\n", "WaterMelon", waterMelonDistance);
  printf("%sDistance: %10.f\n", "BokChoy", bokChoyDistance);
  printf("%sDistance: %10.f\n", "Dorian", dorianDistance);
  printf("%sDistance: %10.f\n", "Melon", melonDistance);
  printf("%sDistance: %10.f\n", "Cucumber", cucumberDistance);
  printf("%sDistance: %10.f\n", "Strawberry", strawberryDistance);
  printf("以上");
  // TODO ここまで消そう
  */

  int pwm;
  float kp;
  float ki;
  float kd;
  int dt;

  int leftPow;
  int rightPow;

  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

// 歌い始める
#ifdef SingASong
  commandExecutor->addCommand(new StartCyc(SING_A_SONG_CYC), new NumberOfTimesPredicate(1), "sing a song");
#endif

  // スタート後メッセージ出力コマンドの初期化とCommandExecutorへの追加
  vector<string> messageLines;
  messageLines.push_back("Started!!");
  messageLines.push_back("GOGOGO!!");
  PrintMessage *printMessage = new PrintMessage(messageLines, true);
  Predicate *printMessagePredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(printMessage, printMessagePredicate, GET_VARIABLE_NAME(printMessage));

  // BananaPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
  calibrator->addPIDTracer(bananaPIDTracer);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 15;
  kp = 0.75;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
  calibrator->addPIDTracer(orangePIDTracer);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

  // StarFruitsWalkerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 20;
  Walker *starFruitsWalker = new Walker(leftPow, rightPow);
  Predicate *predicateStarFruits = new WheelDistancePredicate(starFruitsDistance, robotAPI);
  commandExecutor->addCommand(starFruitsWalker, predicateStarFruits, GET_VARIABLE_NAME(starFruitsWalker));

  // CherryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *cherryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCherry = new WheelDistancePredicate(cherryDistance, robotAPI);
  calibrator->addPIDTracer(cherryPIDTracer);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry, GET_VARIABLE_NAME(cherryPIDTracer));

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
  calibrator->addPIDTracer(waterMelonPIDTracer);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

  // BokChoyWalkerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 18;
  Walker *bokChoyWalker = new Walker(leftPow, rightPow);
  Predicate *predicateBokChoy = new WheelDistancePredicate(bokChoyDistance, robotAPI);
  commandExecutor->addCommand(bokChoyWalker, predicateBokChoy, GET_VARIABLE_NAME(bokChoyWalker));

  // DorianPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *dorianPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateDorian = new WheelDistancePredicate(dorianDistance, robotAPI);
  calibrator->addPIDTracer(dorianPIDTracer);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian, GET_VARIABLE_NAME(dorianPIDTracer));

  // MelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *melonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
  calibrator->addPIDTracer(melonPIDTracer);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

  // CucumberPIDTracerの初期化とCommandExecutorへの追加
  pwm = 24;
  kp = 0.4;
  ki = 0.2;
  kd = 0.4;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
  calibrator->addPIDTracer(cucumberPIDTracer);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateStrawberry = new ColorPredicate(COLOR_BLUE);
  calibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

  // Commandの定義とCommandExecutorへの追加ここまで

#ifdef SimulatorMode
  // シミュレータはPIDTargetBrightnessをキャリブレーションしないので値を設定する必要がある
  int targetBrightness = 20;
  bananaPIDTracer->setTargetBrightness(targetBrightness);
  orangePIDTracer->setTargetBrightness(targetBrightness);
  cherryPIDTracer->setTargetBrightness(targetBrightness);
  waterMelonPIDTracer->setTargetBrightness(targetBrightness);
  dorianPIDTracer->setTargetBrightness(targetBrightness);
  melonPIDTracer->setTargetBrightness(targetBrightness);
  cucumberPIDTracer->setTargetBrightness(targetBrightness);
  strawberryPIDTracer->setTargetBrightness(targetBrightness);
#endif

#ifdef RightCourceMode
  commandExecutor->reverseCommandAndPredicate();
#endif
}
#endif

// DistanceReaderModeの場合のcommandExecutor初期化処理
#ifdef DistanceReaderMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // distanceReaderの初期化とCommandExecutorへの追加
  DistanceReader *distanceReader = new DistanceReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(distanceReader, startButtonPredicate, GET_VARIABLE_NAME(distanceReader));
}
#endif

#ifdef FlatLineMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  float bananaDistance = 109;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  float orangeDistance = 113;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  float starFruitsDistance = 9;   // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  float cherryDistance = 18;      // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  float waterMelonDistance = 300; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  float bokChoyDistance = 35;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  float dorianDistance = 25;      // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  float melonDistance = 209;      // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  float cucumberDistance = 140;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  float strawberryDistance = 140; // ゴールまで。いちご好き。ライントレースする。

  int pwm = 20;
  float kp = 0.7;
  float ki = 0.2;
  float kd = 0.7;
  int dt = 1;

  int leftPow;
  int rightPow;

  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

// 歌い始める
#ifdef SingASong
  commandExecutor->addCommand(new StartCyc(SING_A_SONG_CYC), new NumberOfTimesPredicate(1), "sing a song");
#endif

  // スタート後メッセージ出力コマンドの初期化とCommandExecutorへの追加
  vector<string> messageLines;
  messageLines.push_back("Started!!");
  messageLines.push_back("GOGOGO!!");
  PrintMessage *printMessage = new PrintMessage(messageLines, true);
  Predicate *printMessagePredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(printMessage, printMessagePredicate, GET_VARIABLE_NAME(printMessage));

  // BananaPIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
  calibrator->addPIDTracer(bananaPIDTracer);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
  calibrator->addPIDTracer(orangePIDTracer);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

  // StarFruitsWalkerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 20;
  Walker *starFruitsWalker = new Walker(leftPow, rightPow);
  Predicate *predicateStarFruits = new WheelDistancePredicate(starFruitsDistance, robotAPI);
  commandExecutor->addCommand(starFruitsWalker, predicateStarFruits, GET_VARIABLE_NAME(starFruitsWalker));

  // CherryPIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *cherryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCherry = new WheelDistancePredicate(cherryDistance, robotAPI);
  calibrator->addPIDTracer(cherryPIDTracer);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry, GET_VARIABLE_NAME(cherryPIDTracer));

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
  calibrator->addPIDTracer(waterMelonPIDTracer);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

  // BokChoyWalkerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 18;
  Walker *bokChoyWalker = new Walker(leftPow, rightPow);
  Predicate *predicateBokChoy = new WheelDistancePredicate(bokChoyDistance, robotAPI);
  commandExecutor->addCommand(bokChoyWalker, predicateBokChoy, GET_VARIABLE_NAME(bokChoyWalker));

  // DorianPIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *dorianPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateDorian = new WheelDistancePredicate(dorianDistance, robotAPI);
  calibrator->addPIDTracer(dorianPIDTracer);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian, GET_VARIABLE_NAME(dorianPIDTracer));

  // MelonPIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *melonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
  calibrator->addPIDTracer(melonPIDTracer);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

  // CucumberPIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
  calibrator->addPIDTracer(cucumberPIDTracer);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
  calibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

  // Commandの定義とCommandExecutorへの追加ここまで

#ifdef SimulatorMode
  // シミュレータはPIDTargetBrightnessをキャリブレーションしないので値を設定する必要がある
  int targetBrightness = 20;
  bananaPIDTracer->setTargetBrightness(targetBrightness);
  orangePIDTracer->setTargetBrightness(targetBrightness);
  cherryPIDTracer->setTargetBrightness(targetBrightness);
  waterMelonPIDTracer->setTargetBrightness(targetBrightness);
  dorianPIDTracer->setTargetBrightness(targetBrightness);
  melonPIDTracer->setTargetBrightness(targetBrightness);
  cucumberPIDTracer->setTargetBrightness(targetBrightness);
  strawberryPIDTracer->setTargetBrightness(targetBrightness);
#endif

#ifdef RightCourceMode
  commandExecutor->reverseCommandAndPredicate();
#endif
}
#endif

#ifdef SlalomUFOTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  Stopper *stopper = new Stopper();

  int pwm;
  int leftPWM;
  int rightPWM;

  float distance;

  float n;
  int walkerPWM;
  int rotatePWM;
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
  CurvatureWalkerCommandAndPredicate *curvatureWalkerCommandAndPredicate1 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
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

#if defined(SlalomAwaitingSignalModePattern1_1) | defined(SlalomAwaitingSignalModePattern2_1)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();

  int pwm;
  float kp;
  float ki;
  float kd;
  float dt;

  float r;
  float theta;

  float angle;

  int leftPWM;
  int rightPWM;

  int numberOfTime;

  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);

  // Calibrator
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

// 歌い始める
#ifdef SingASong
  commandExecutor->addCommand(new StartCyc(SING_A_SONG_CYC), new NumberOfTimesPredicate(1), "sing a song");
#endif

  // スラローム進入ここから
  // コース上2つ目の青線前から開始。

  pwm = 15;
  kp = 0.2;
  ki = 0.1;
  kd = 0.2;
  dt = 1;
  ColorPIDTracer *pidTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 8;
  ColorPIDTracer *lowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  calibrator->addColorPIDTracer(pidTracer);
  calibrator->addColorPIDTracer(lowPWMTracer);

#ifdef SimulatorMode
  float targetBrightness = 20;
  rgb_raw_t targetRGB;
  targetRGB.r = 110;
  targetRGB.g = 100;
  targetRGB.b = 150;
  pidTracer->setTargetColor(targetRGB);
  lowPWMTracer->setTargetColor(targetRGB);
#endif

  // PIDトレースで青線まで進む
  Predicate *pidTracerPredicate = new ColorPredicate(COLOR_BLUE);
  // TODO commandExecutor->addCommand(pidTracer, pidTracerPredicate, GET_VARIABLE_NAME(pidTracer));

  // スラローム直前までPIDトレース
  float distance = 30;
  // TODO commandExecutor->addCommand(pidTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // アームを下げる
  int armAngle = 15;
  pwm = -10;
  numberOfTime = 25;
  Command *armDown = new ArmController(pwm);
  Predicate *armDownPredicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(armDown, armDownPredicate, GET_VARIABLE_NAME(armDown));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // テールモータで角度をつける
  pwm = 50;
  numberOfTime = 25;
  Command *tailMotorDrive = new TailController(pwm);
  Predicate *tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

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
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
  Command *back = new Walker(-5, -5);
  distance = -3;
  Predicate *backPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(back, backPredicate, GET_VARIABLE_NAME(back));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // アームを戻す
  pwm = 10;
  Command *armUp = new ArmController(pwm);
  Predicate *armUpPredicate = new MotorRotateAnglePredicate(armAngle, robotAPI->getArmMotor());
  commandExecutor->addCommand(armUp, armUpPredicate, GET_VARIABLE_NAME(armUp));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // テールモータを戻す
  pwm = 30;
  numberOfTime = 25;
  tailMotorDrive = new TailController(-pwm);
  tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  distance = 27;
  Walker *walker1 = new Walker(20, 20);
  commandExecutor->addCommand(walker1, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(walker1));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // スラローム進入ここまで

  // 指示待ち走行ここから

  // 45度左旋回
  pwm = 10;
  angle = -45;
  RotateRobotUseGyroCommandAndPredicate *rotate1 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate1->getCommand(), rotate1->getPredicate(), GET_VARIABLE_NAME(rotate1));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 13.5;
  Walker *walker2 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker2Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker2, walker2Predicate, GET_VARIABLE_NAME(walker2));

  // 49.5度右旋回
  pwm = 10;
  angle = 49.5;
  RotateRobotUseGyroCommandAndPredicate *rotate2 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate2->getCommand(), rotate2->getPredicate(), GET_VARIABLE_NAME(rotate2));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 10;
  rightPWM = 10;
#endif
  distance = 15;
  Walker *walker3 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker3Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker3, walker3Predicate, GET_VARIABLE_NAME(walker3));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 7;
#endif
  r = 31;
  theta = 25;
  CurvatureWalkerCommandAndPredicate *curve1 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve1->getCommand(), curve1->getPredicate(), GET_VARIABLE_NAME(curve1));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 16;
  Walker *walker4 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker4Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker4, walker4Predicate, GET_VARIABLE_NAME(walker4));

  // 60度左旋回
#ifdef SimulatorMode
  pwm = 10;
#else
  pwm = 6;
#endif
  angle = -60;
  RotateRobotUseGyroCommandAndPredicate *rotate3 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate3->getCommand(), rotate3->getPredicate(), GET_VARIABLE_NAME(rotate3));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 13;
  Walker *walker5 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 8;
#endif
  r = 16;
  theta = 20;
  CurvatureWalkerCommandAndPredicate *curve2 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve2->getCommand(), curve2->getPredicate(), GET_VARIABLE_NAME(curve2));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 8;
#endif
  r = 8;
  theta = 35;
  CurvatureWalkerCommandAndPredicate *curve3 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve3->getCommand(), curve3->getPredicate(), GET_VARIABLE_NAME(curve3));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 17.5;
  Walker *walker6 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker6Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker6, walker6Predicate, GET_VARIABLE_NAME(walker6));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 10;
#endif
  r = 16;
  theta = -20;
  CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 18;
  Walker *walker7 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));

  // ガレージカード色取得
  commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));

#ifdef SlalomAwaitingSignalModePattern1_1
  // 85度左旋回
  pwm = 10;
  angle = -85;
  RotateRobotUseGyroCommandAndPredicate *rotate4 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate4->getCommand(), rotate4->getPredicate(), GET_VARIABLE_NAME(rotate4));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 33;
  Walker *walker8 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker8Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker8, walker8Predicate, GET_VARIABLE_NAME(walker8));

  // 90度左旋回
#ifdef SimulatorMode
  pwm = 10;
#else
  pwm = 10;
#endif
  angle = -90;
  RotateRobotUseGyroCommandAndPredicate *rotate5 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate5->getCommand(), rotate5->getPredicate(), GET_VARIABLE_NAME(rotate5));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 7;
#endif
  r = 13.5;
  theta = 90;
  CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 20;
  Walker *walker10 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker10Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker10, walker10Predicate, GET_VARIABLE_NAME(walker10));

  // 45度右旋回
  pwm = 10;
  angle = 45;
  RotateRobotUseGyroCommandAndPredicate *rotate6 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate6->getCommand(), rotate6->getPredicate(), GET_VARIABLE_NAME(rotate6));

  //  直進。黒を拾うまで
  leftPWM = 10;
  rightPWM = 10;
  distance = 15;
  Walker *walker9 = new Walker(leftPWM, rightPWM);
  Predicate *walker9Predicate = new ColorPredicate(COLOR_BLACK);
  commandExecutor->addCommand(walker9, walker9Predicate, GET_VARIABLE_NAME(walker9));

  // PIDで青線まで進む
  commandExecutor->addCommand(lowPWMTracer, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(lowPWMTracer));

#endif

#ifdef SlalomAwaitingSignalModePattern2_1
  // 90度左回転
  pwm = 10;
  angle = -90;
  RotateRobotUseGyroCommandAndPredicate *rotate4 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate4->getCommand(), rotate4->getPredicate(), GET_VARIABLE_NAME(rotate4));

  // 30度左回転
  pwm = 10;
  angle = -30;
  RotateRobotUseGyroCommandAndPredicate *rotate5 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate5->getCommand(), rotate5->getPredicate(), GET_VARIABLE_NAME(rotate4));

  //  直進
  leftPWM = 15;
  rightPWM = 15;
  distance = 13.5;
  Walker *walker8 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker8Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker8, walker8Predicate, GET_VARIABLE_NAME(walker8));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 7;
#endif
  r = 23;
  theta = 50;
  CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 7;
#endif
  r = 15;
  theta = -65;
  CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));

  // PIDで青線まで進む
  commandExecutor->addCommand(lowPWMTracer, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(lowPWMTracer));

  // 指示待ち走行ここまで
#endif

  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#if defined(SlalomAwaitingSignalModePattern1_2) | defined(SlalomAwaitingSignalModePattern2_2)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();

  int pwm;
  float kp;
  float ki;
  float kd;
  float dt;

  float r;
  float theta;

  float angle;

  int leftPWM;
  int rightPWM;

  int numberOfTime;

  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);

  // Calibrator
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

// 歌い始める
#ifdef SingASong
  commandExecutor->addCommand(new StartCyc(SING_A_SONG_CYC), new NumberOfTimesPredicate(1), "sing a song");
#endif

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
  float targetBrightness = 20;
  rgb_raw_t targetRGB;
  targetRGB.r = 60;
  targetRGB.g = 60;
  targetRGB.b = 60;
  pidTracer->setTargetColor(targetRGB);
  lowPWMTracer->setTargetColor(targetRGB);
#endif

  // PIDトレースで青線まで進む
  Predicate *pidTracerPredicate = new BluePredicate();
  commandExecutor->addCommand(pidTracer, pidTracerPredicate, GET_VARIABLE_NAME(pidTracer));

  // スラローム直前までPIDトレース
  float distance = 30;
  commandExecutor->addCommand(pidTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // アームを下げる
  int armAngle = 15;
  pwm = -10;
  numberOfTime = 25;
  Command *armDown = new ArmController(pwm);
  Predicate *armDownPredicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(armDown, armDownPredicate, GET_VARIABLE_NAME(armDown));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // テールモータで角度をつける
  pwm = 50;
  numberOfTime = 25;
  Command *tailMotorDrive = new TailController(pwm);
  Predicate *tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

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
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
  Command *back = new Walker(-5, -5);
  distance = -3;
  Predicate *backPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(back, backPredicate, GET_VARIABLE_NAME(back));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // アームを戻す
  pwm = 10;
  Command *armUp = new ArmController(pwm);
  Predicate *armUpPredicate = new MotorRotateAnglePredicate(armAngle, robotAPI->getArmMotor());
  commandExecutor->addCommand(armUp, armUpPredicate, GET_VARIABLE_NAME(armUp));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // テールモータを戻す
  pwm = 30;
  numberOfTime = 25;
  tailMotorDrive = new TailController(-pwm);
  tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  distance = 27;
  Walker *walker1 = new Walker(20, 20);
  commandExecutor->addCommand(walker1, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(walker1));
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));

  // スラローム進入ここまで

  // 指示待ち走行ここから

  // 45度左旋回
  pwm = 10;
  angle = -45;
  RotateRobotUseGyroCommandAndPredicate *rotate1 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate1->getCommand(), rotate1->getPredicate(), GET_VARIABLE_NAME(rotate1));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 13.5;
  Walker *walker2 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker2Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker2, walker2Predicate, GET_VARIABLE_NAME(walker2));

  // 45度右旋回
  pwm = 10;
  angle = 45;
  RotateRobotUseGyroCommandAndPredicate *rotate2 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate2->getCommand(), rotate2->getPredicate(), GET_VARIABLE_NAME(rotate2));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 13.5; // TODO
  Walker *walker3 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker3Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker3, walker3Predicate, GET_VARIABLE_NAME(walker3));

  // 45度右旋回
  pwm = 10;
  angle = 45;
  RotateRobotUseGyroCommandAndPredicate *rotate3 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate3->getCommand(), rotate3->getPredicate(), GET_VARIABLE_NAME(rotate3));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 4; // TODO
  Walker *walker4 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker4Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker4, walker4Predicate, GET_VARIABLE_NAME(walker4));

  // 45度左旋回
  pwm = 10;
  angle = -45;
  RotateRobotUseGyroCommandAndPredicate *rotate4 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate4->getCommand(), rotate4->getPredicate(), GET_VARIABLE_NAME(rotate4));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 8; // TODO
  Walker *walker5 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));

  // 90度右旋回
  pwm = 10;
  angle = 90;
  RotateRobotUseGyroCommandAndPredicate *rotate5 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate5->getCommand(), rotate5->getPredicate(), GET_VARIABLE_NAME(rotate5));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 16; // TODO
  Walker *walker6 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker6Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker6, walker6Predicate, GET_VARIABLE_NAME(walker6));

  // 20度左旋回
  pwm = 10;
  angle = -20;
  RotateRobotUseGyroCommandAndPredicate *rotate6 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate6->getCommand(), rotate6->getPredicate(), GET_VARIABLE_NAME(rotate6));

  // 直進
  leftPWM = 16;
  rightPWM = 16;
  distance = 16; // TODO
  Walker *walker7 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));
}
#endif

#ifdef BlockTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // TODO 実装して

  // 停止コマンドの初期化とCommandExecutorへの追加
  int numberOfTimes = 1;
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(numberOfTimes);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

// RGBRawReaderModeの場合のcommandExecutor初期化処理
#ifdef RGBRawReaderMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // rgbRawReaderの初期化とCommandExecutorへの追加
  RGBRawReader *rgbRawReader = new RGBRawReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(rgbRawReader, startButtonPredicate, GET_VARIABLE_NAME(rgbRawReader));
}
#endif

// Rotate360TestModeの場合のcommandExecutor初期化処理
#if defined(Rotate360TestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  int motorRotateAngle = 540; // ここの値をいじってはかって

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  Walker *walker = new Walker(pwm, -pwm); // 右に向く
  Predicate *walkerPredicate = new MotorCountPredicate(robotAPI->getLeftWheel(), motorRotateAngle);
  commandExecutor->addCommand(walker, walkerPredicate, GET_VARIABLE_NAME(walker));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

// RotateTestModeの場合のcommandExecutor初期化処理
#if defined(RotateTestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int angle = 10;
  int pwm = 15;
  RotateRobotCommandAndPredicate *commandAndPredicate = new RotateRobotCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), "rotateRobot");

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

// NOTE ジャイロ、 実機とシミュレータで左右判定が逆になる？
#if defined(RotateGyroTestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  int angle = -30;
  RotateRobotUseGyroCommandAndPredicate *rotateRobotCommandAndPredicate = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotateRobotCommandAndPredicate->getCommand(), rotateRobotCommandAndPredicate->getPredicate(), "rotateRobot");

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#ifdef StraightTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, GET_VARIABLE_NAME(stopper)); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

// 歌い始める
#ifdef SingASong
  commandExecutor->addCommand(new StartCyc(SING_A_SONG_CYC), new NumberOfTimesPredicate(1), "sing a song");
#endif

  // 直進コマンドの初期化とCommandExecutorへの追加
  int pwm = 50;
  float distanceCm = 10000;
  Walker *walker = new Walker(pwm, pwm);
  WheelDistancePredicate *walkerPredicate = new WheelDistancePredicate(distanceCm, robotAPI);
  commandExecutor->addCommand(walker, walkerPredicate, GET_VARIABLE_NAME(walker));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#if defined(CurvatureWalkerTestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 曲率進行コマンドの初期化とCommandExecutorへの追加
  int pwm = 20; // NOTE pwm上げるとおかしくなる
  float r = 30;
  float theta = 90;
  CurvatureWalkerCommandAndPredicate *commandAndPredicate = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), "curvatureWalker");

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#if defined(SwingSonarDetectorTestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 障害物検出コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  float swingLeft = 90.0;
  float swingRight = -90.0;
  int targetLeft = 20;
  int targetRight = 20;
  SwingSonarObstacleDetector *swingSonarDetector = new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT_CENTER, pwm, swingLeft, swingRight, targetLeft, targetRight);
  Predicate *swingSonarDetectorPredicate = new FinishedCommandPredicate(swingSonarDetector);
  commandExecutor->addCommand(swingSonarDetector, swingSonarDetectorPredicate, GET_VARIABLE_NAME(swingSonarDetector));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#if defined(ShigekiTestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  int pwm = 10;
  float acn = -30.91474484;
  float nc = 23.72114075;
  // float bcn = 1.022709978;
  float nTurn = 69.07498194;
  float n = 1.0;

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // ACN度回転する
  RotateRobotCommandAndPredicate *turnACNCommandAndPredicate = new RotateRobotCommandAndPredicate(acn, pwm, robotAPI);
  commandExecutor->addCommand(turnACNCommandAndPredicate->getCommand(), turnACNCommandAndPredicate->getPredicate(), "turnACN");

  // NCの距離進む
  Walker *walkNCCommand = new Walker(pwm, pwm);
  WheelDistancePredicate *walkNCPredicate = new WheelDistancePredicate(nc, robotAPI);
  commandExecutor->addCommand(walkNCCommand, walkNCPredicate, GET_VARIABLE_NAME(walkNCCommand));

  // nTurn分旋回する
  RotateRobotCommandAndPredicate *turnNCommandAndPredicate = new RotateRobotCommandAndPredicate(nTurn, pwm, robotAPI);
  commandExecutor->addCommand(turnNCommandAndPredicate->getCommand(), turnNCommandAndPredicate->getPredicate(), "turnN");

  // n分進む
  Walker *walkNCommand = new Walker(pwm, pwm);
  WheelDistancePredicate *walkNPredicate = new WheelDistancePredicate(n, robotAPI);
  commandExecutor->addCommand(walkNCommand, walkNPredicate, GET_VARIABLE_NAME(walkNCommand));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#if defined(UFORunnerSwingTestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

// UFO走行コマンドの初期化とCommandExecutorへの追加
#ifdef SimulatorMode
  float n = 8;
  int walkerPWM = 20;
  int rotatePWM = 5;

  float swingLeftAngle = -90.0;
  float swingRightAngle = 90.0;

  int targetLeftDistance = 30;
  int targetRightDistance = 10;
  bool reverseTest = false;
#else
  float n = 5;
  int walkerPWM = 10;
  int rotatePWM = 4;

  float swingLeftAngle = -90.0;
  float swingRightAngle = 90.0;

  int targetLeftDistance = 30;
  int targetRightDistance = 30;
  bool reverseTest = false;
#endif

  UFORunner *ufoRunner = new UFORunner(n, walkerPWM, rotatePWM, swingLeftAngle, swingRightAngle, targetLeftDistance, targetRightDistance);
  if (reverseTest)
  {
    ufoRunner = ufoRunner->generateReverseCommand();
  }
  commandExecutor->addCommand(ufoRunner, new FinishedCommandPredicate(ufoRunner), GET_VARIABLE_NAME(ufoRunner));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#if defined(UFORunnerClockwiseTestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // UFO走行コマンドの初期化とCommandExecutorへの追加
#ifdef SimulatorMode
  float n = 5;
  int walkerPWM = 15;
  int rotatePWM = 3;

  float angle = 180;
  int targetLeftDistance = 20;  // これを検知した状態からはじめて
  int targetRightDistance = 20; // あとはSwingSonarと同じ

  int skipFrameAfterDetectFirstObstacle = 0;
  bool facingObstacle = true;
  bool reverseTest = true;
#else
  float n = 4;
  int walkerPWM = 15;
  int rotatePWM = 5;

  float angle = 180;
  int targetLeftDistance = 40;  // これを検知した状態からはじめて
  int targetRightDistance = 40; // あとはSwingSonarと同じ

  int skipFrameAfterDetectFirstObstacle = 10;
  bool facingObstacle = false;
  bool reverseTest = true;
#endif

  UFORunner *ufoRunner = new UFORunner(n, walkerPWM, rotatePWM, angle, targetLeftDistance, targetRightDistance, skipFrameAfterDetectFirstObstacle, facingObstacle);
  if (reverseTest)
  {
    ufoRunner = ufoRunner->generateReverseCommand();
  }
  commandExecutor->addCommand(ufoRunner, new FinishedCommandPredicate(ufoRunner), GET_VARIABLE_NAME(ufoRunner));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#ifdef BrightnessPIDTracerTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  int pwm = 20;
  float kp = 0.7;
  float ki = 0.2;
  float kd = 0.7;
  float dt = 1;
  PIDTracer *pidTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  commandExecutor->addCommand(pidTracer, new Predicate(), GET_VARIABLE_NAME(pidTracer));
  calibrator->addPIDTracer(pidTracer);
}
#endif

#ifdef ColorPIDTracerTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  int pwm = 20;
  float kp = 0.2;
  float ki = 0.1;
  float kd = 0.2;
  float dt = 1;
  ColorPIDTracer *colorPIDTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  commandExecutor->addCommand(colorPIDTracer, new Predicate(), GET_VARIABLE_NAME(colorPIDTracer));
  calibrator->addColorPIDTracer(colorPIDTracer);

#ifdef SimulatorMode
  rgb_raw_t targetRGB;
  targetRGB.r = 110;
  targetRGB.g = 100;
  targetRGB.b = 150;
  colorPIDTracer->setTargetColor(targetRGB);
#endif
}
#endif

#ifdef FroggySongTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  vector<Note *> froggySong = generateFroggySong();
  for (int i = 0; i < ((int)froggySong.size()); i++)
  {
    commandExecutor->addCommand(froggySong[i], new FinishedCommandPredicate(froggySong[i]), "");
  }
  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "stopper");
}

#endif

void runner_task(intptr_t exinf)
{
  ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示
  commandExecutor->run();                         // 走らせる
  ext_tsk();
}

// TODO コードの場所移動して
enum ReturnToStartPointState
{
  RTSP_TURNNING_UP,
  RTSP_WALKING_UP,
  RTSP_TURNNING_RIGHT,
  RTSP_WALKING_RIGHT,
  RTSP_FINISH,
};

// TODO コードの場所移動して
ReturnToStartPointState returnToStartPointState = RTSP_TURNNING_UP;
Walker *returnToStartPointStraightWalker = new Walker(20, 20);
Walker *returnToStartPointTurnRightWalker = new Walker(10, -10);
Walker *returnToStartPointTurnLeftWalker = new Walker(-10, 10);
colorid_t returnToStartPointEdgeLineColor = COLOR_RED;

void return_to_start_point_task(intptr_t exinf)
{
  vector<string> messageLines;
  messageLines.push_back("STARTED Back to the future");
  PrintMessage *printMessage = new PrintMessage(messageLines, true);
  printMessage->run(robotAPI);
  delete printMessage;

  switch (returnToStartPointState)
  {
  case RTSP_TURNNING_UP:
  {
    int targetAngle = 180;
#ifdef SimulatorMode
    int angle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    int angle = robotAPI->getGyroSensor()->getAngle();
#endif

    if (angle > targetAngle)
    {
      returnToStartPointTurnRightWalker->run(robotAPI);
    }
    else
    {
      returnToStartPointTurnLeftWalker->run(robotAPI);
    }

    // これのためだけにPredicate定義するのは嫌なので筋肉コーディングします
    if (angle > targetAngle - 5 && angle < targetAngle + 5)
    {
      returnToStartPointState = RTSP_WALKING_UP;
    }
  }
  break;

  case RTSP_WALKING_UP:
  {
    returnToStartPointStraightWalker->run(robotAPI);
    colorid_t colorID = robotAPI->getColorSensor()->getColorNumber();
    if (colorID == returnToStartPointEdgeLineColor)
    {
      returnToStartPointState = RTSP_TURNNING_RIGHT;
    }
  }
  break;

  case RTSP_TURNNING_RIGHT:
  {
    int targetAngle = 90;
#ifdef SimulatorMode
    int angle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    int angle = robotAPI->getGyroSensor()->getAngle();
#endif

    if (angle > targetAngle)
    {
      returnToStartPointTurnRightWalker->run(robotAPI);
    }
    else
    {
      returnToStartPointTurnLeftWalker->run(robotAPI);
    }

    // これのためだけにPredicate定義するのは嫌なので筋肉コーディングします
    if (angle > targetAngle - 5 && angle < targetAngle + 5)
    {
      returnToStartPointState = RTSP_WALKING_RIGHT;
    }
  }
  break;

  case RTSP_WALKING_RIGHT:
  {
    returnToStartPointStraightWalker->run(robotAPI);
  }
  break;

  case RTSP_FINISH:
  {
    // 別になくてもいっか。スタート地点に帰ってくればいいんだからな。
  }
  break;

  default:
    break;
  }
  ext_tsk();
}

// TODO コードの場所移動して
enum BTCommand
{
  BTC_EMERGENCY_STOP = 's',
  BTC_RETURN_TO_START_POINT = 'r',
};

void listen_bluetooth_command_task(intptr_t exinf)
{
#ifdef EnableBluetooth
  const uint32_t sleepDuration = 100 * 1000;

  unsigned char bluetoothCommand = fgetc(bt);
  switch (bluetoothCommand)
  {
  case BTC_EMERGENCY_STOP:
  {
    stp_cyc(SING_A_SONG_CYC);
    stp_cyc(RETURN_TO_START_POINT_CYC);
    commandExecutor->emergencyStop();
    break;
  }
  case BTC_RETURN_TO_START_POINT:
  {
    commandExecutor->emergencyStop();

    // runnerTaskが終了するのを待機する
    for (; true; robotAPI->getClock()->sleep(sleepDuration))
    {
      T_RCYC pk_rcyc;
      ref_cyc(RUNNER_CYC, &pk_rcyc);
      if (pk_rcyc.cycstat == TCYC_STP)
      {
        break;
      }
    }
    sta_cyc(RETURN_TO_START_POINT_CYC);
    break;
  }
  default:
    break;
  }

  if (ev3_button_is_pressed(LEFT_BUTTON))
  {
    stp_cyc(LISTEN_BLUETOOTH_COMMAND_CYC);
  }
#endif
  ext_tsk();
}

CommandExecutor *singASongCommandExecutor;
void sing_a_song_task(intptr_t exinf)
{
  singASongCommandExecutor->run();
  ext_tsk();
}

void initSong(int loop)
{
  for (int j = 0; j < loop; j++)
  {
    vector<Note *> song = generateFroggySong();
    for (int i = 0; i < ((int)song.size()); i++)
    {
      singASongCommandExecutor->addCommand(song[i], new FinishedCommandPredicate(song[i]), "");
    }
  }
}

void main_task(intptr_t unused)
{
  const uint32_t sleepDuration = 100 * 1000;

  // EV3APIオブジェクトの初期化
  TouchSensor *touchSensor = new TouchSensor(PORT_1);
  ColorSensor *colorSensor = new ColorSensor(PORT_2);
  SonarSensor *sonarSensor = new SonarSensor(PORT_3);
  GyroSensor *gyroSensor = new GyroSensor(PORT_4);
  Motor *armMotor = new Motor(PORT_A);
  Motor *rightWheel = new Motor(PORT_B);
  Motor *leftWheel = new Motor(PORT_C);
  Motor *tailMotor = new Motor(PORT_D);
  Clock *clock = new Clock();

  // RobotAPIとCommandExecutorの初期化
  robotAPI = new RobotAPI(touchSensor, colorSensor, sonarSensor, leftWheel, rightWheel, armMotor, tailMotor, gyroSensor, clock);
  commandExecutor = new CommandExecutor(robotAPI, true);

  // ev3_lcd_set_font(EV3_FONT_MEDIUM);           // フォントの設定
  ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示

  // robotAPIの初期化。完全停止してapiを初期化する
  Stopper *stopper = new Stopper();
  stopper->run(robotAPI);
  delete stopper;
  robotAPI->reset();
  vector<string> resetedMessageLines;
  resetedMessageLines.push_back("reseted api");
  PrintMessage *printResetedMessage = new PrintMessage(resetedMessageLines, true);
  printResetedMessage->run(robotAPI);
  delete printResetedMessage;

  // commandExecutorを初期化する（挙動定義）
  initializeCommandExecutor(commandExecutor, robotAPI);
  vector<string> readyMessageLines;
  readyMessageLines.push_back("ready");
  PrintMessage *printReadyMessage = new PrintMessage(resetedMessageLines, true);
  printReadyMessage->run(robotAPI);
  delete printReadyMessage;

// FroggySongを歌うCommandExecutorを初期化する
#ifdef SingASong
  singASongCommandExecutor = new CommandExecutor(robotAPI, false);
  initSong(loopSong);
#endif

  // commandExecutor->run()の周期ハンドラを起動する
  sta_cyc(RUNNER_CYC);

  // bluetoothCommandを受け取る周期ハンドラを起動する
  sta_cyc(LISTEN_BLUETOOTH_COMMAND_CYC);

  // 終了判定処理
  for (; true; clock->sleep(sleepDuration))
  {
    // 左ボタンが押されたら緊急停止のためにループを抜ける
    if (ev3_button_is_pressed(LEFT_BUTTON))
    {
      // 停止処理
      commandExecutor->emergencyStop();
#ifdef SingASong
      singASongCommandExecutor->emergencyStop();
#endif
      break;
    }

    // RUNNER_CYCが終了していたら走行完了なのでループを抜ける
    T_RCYC runnerCycState;
    T_RCYC btcCycState;
    ref_cyc(RUNNER_CYC, &runnerCycState);
    ref_cyc(LISTEN_BLUETOOTH_COMMAND_CYC, &btcCycState);
    if (runnerCycState.cycstat == TCYC_STP && btcCycState.cycstat == TCYC_STP)
    {
      Stopper *stopper = new Stopper();
      stopper->run(robotAPI);
      delete stopper;

      vector<string> messageLines;
      messageLines.push_back("finish!!");
      PrintMessage printFinishMessage(messageLines, true);
      printFinishMessage.run(robotAPI);
      break;
    }
  }

#ifdef EnableBluetooth
  // RUNNER_CYCが走っていたら止める
  T_RCYC pk_rcyc;
  ref_cyc(RETURN_TO_START_POINT_CYC, &pk_rcyc);
  if (pk_rcyc.cycstat == TCYC_STA)
  {
    stp_cyc(RETURN_TO_START_POINT_CYC);
  }
#endif

#ifdef SingASong
  // 歌ってたら止める
  stp_cyc(SING_A_SONG_CYC);
#endif

  // メインタスクの終了
  ext_tsk();

  // オブジェクトの削除
  delete commandExecutor;
  delete robotAPI;
  delete touchSensor;
  delete colorSensor;
  delete sonarSensor;
  delete gyroSensor;
  delete armMotor;
  delete leftWheel;
  delete rightWheel;
  delete tailMotor;
  delete clock;
  delete returnToStartPointStraightWalker;
  delete returnToStartPointTurnLeftWalker;
  delete returnToStartPointTurnRightWalker;
}