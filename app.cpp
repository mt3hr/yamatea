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

using namespace std;
using namespace ev3api;

CommandExecutor *commandExecutor;
RobotAPI *robotAPI;

void stp_cyc_all()
{
  stp_cyc(RUNNER_CYC);
  stp_cyc(SING_A_SONG_CYC);
  stp_cyc(DEDEDON_CYC);
  stp_cyc(RETURN_TO_START_POINT_CYC);
  stp_cyc(LISTEN_BLUETOOTH_COMMAND_CYC);
}

// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#ifdef GoalSanekataPIDMode1
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

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

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
  Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
  calibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

// Commandの定義とCommandExecutorへの追加ここまで

// シミュレータはPIDTargetBrightnessをキャリブレーションしないので値を設定する必要がある
#if defined(SimulatorMode)
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

  float pwm = 20;
  float kp = 0.7;
  float ki = 0.2;
  float kd = 0.7;
  float dt = 1;

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

#if defined(SimulatorMode)
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

#ifdef GoalOkiharaPIDMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
}
#endif

// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#ifdef GoalOkiharaPIDMode1
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  /*
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
  */

  int sceneBananaMotorCountPredicateArg = 1750;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int sceneOrangeMotorCountPredicateArg = 2500;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2550; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2700;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
#ifdef RightCourceOkiharaMode1
  int sceneBokChoyMotorCountPredicateArg = 6700; // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
#else
  int sceneBokChoyMotorCountPredicateArg = 6490;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
#endif
#ifdef RightCourceOkiharaMode1
  int sceneDorianMotorCountPredicateArg = 6860; // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
#else
  int sceneDorianMotorCountPredicateArg = 6650;      // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
#endif
#ifdef RightCourceOkiharaMode1
  int sceneAsparagusMotorCountPredicateArg = 7310; // ドリアン終了後の１つ目の直線
#else
  int sceneAsparagusMotorCountPredicateArg = 7100;   // ドリアン終了後の１つ目の直線
#endif
#ifdef RightCourceOkiharaMode1
  int sceneRadishMotorCountPredicateArg = 7760; // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
#else
  int sceneRadishMotorCountPredicateArg = 7550;      // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
#endif
#ifdef RightCourceOkiharaMode1
  int sceneMelonMotorCountPredicateArg = 9030; // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
#else
  int sceneMelonMotorCountPredicateArg = 8850;       // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
#endif
#ifdef RightCourceOkiharaMode1
  int sceneCucumberMotorCountPredicateArg = 10705; // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
#else
  int sceneCucumberMotorCountPredicateArg = 10495;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
#endif
#ifdef RightCourceOkiharaMode1
  int sceneStrawberryMotorCountPredicateArg = 11310; // ゴールまで。いちご好き。ライントレースする。
#else
  int sceneStrawberryMotorCountPredicateArg = 11100; // ゴールまで。いちご好き。ライントレースする。
#endif
#ifdef RightCourceOkiharaMode1
  int sceneCabbageMotorCountpredicateArg = 12710; // ゴールまで。
#else
  int sceneCabbageMotorCountpredicateArg = 12500;    // ゴールまで。
#endif
  float distanceTemp = 0;
  int bananaDistance = (sceneBananaMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += bananaDistance;
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
  int cucumberDistance = (sceneCucumberMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cucumberDistance;
  int strawberryDistance = (sceneStrawberryMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += strawberryDistance;
  int cabbageDistance = (sceneCabbageMotorCountpredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cabbageDistance;

  /*
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
    */

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

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

  // BananaPIDTracerの初期化とCommandExecutorへの追加
  pwm = 22;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
  calibrator->addPIDTracer(bananaPIDTracer);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 25;
  kp = 0.5;
  ki = 0.2;
  kd = 0.45;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
  calibrator->addPIDTracer(orangePIDTracer);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

  // StarFruitsWalkerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 16;
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
  pwm = 25;
  kp = 0.5;
  ki = 0.2;
  kd = 0.5;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
  calibrator->addPIDTracer(waterMelonPIDTracer);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

  // BokChoyWalkerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 20;
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

  // AsparagusPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *asparagusPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateAsparagus = new WheelDistancePredicate(asparagusDistance, robotAPI);
  calibrator->addPIDTracer(asparagusPIDTracer);
  commandExecutor->addCommand(asparagusPIDTracer, predicateAsparagus, GET_VARIABLE_NAME(asparagusPIDTracer));

  // RadishPIDTracerの初期化とCommandExecutorへの追加
  pwm = 25;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *radishPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateRadish = new WheelDistancePredicate(radishDistance, robotAPI);
  calibrator->addPIDTracer(radishPIDTracer);
  commandExecutor->addCommand(radishPIDTracer, predicateRadish, GET_VARIABLE_NAME(radishPIDTracer));

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

  pwm = 30;
  kp = 0.5;
  ki = 0.2;
  kd = 0.5;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
  calibrator->addPIDTracer(cucumberPIDTracer);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
  calibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

  // CabbagePIDTracerの初期化とCommandExecutorへの追加
  pwm = 30;
  kp = 0.5;
  ki = 0.2;
  kd = 0.5;
  dt = 1;
  PIDTracer *cabbagePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
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

// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#ifdef GoalOkiharaPIDMode2
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  /*
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
  */

  int sceneBananaMotorCountPredicateArg = 1750;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int sceneOrangeMotorCountPredicateArg = 2500;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2550; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2750;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  int sceneBokChoyMotorCountPredicateArg = 6475;    // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 6650;     // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneAsparagusMotorCountPredicateArg = 7100;
  int sceneRadishMotorCountPredicateArg = 7550;      //ドリアン終了後メロンのカーブ手前までのストレート
  int sceneMelonMotorCountPredicateArg = 8850;       // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneCucumberMotorCountPredicateArg = 10495;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 11100; // いちご好き。ライントレースする。
  int sceneCabbageMotorCountpredicateArg = 12500;    //ゴールまで。

  float distanceTemp = 0;
  int bananaDistance = (sceneBananaMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += bananaDistance;
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
  int cucumberDistance = (sceneCucumberMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cucumberDistance;
  int strawberryDistance = (sceneStrawberryMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += strawberryDistance;
  int cabbageDistance = (sceneCabbageMotorCountpredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cabbageDistance;
  /*
    printf("以下出力された値をbananaDistanceとかに入れていって。");
    printf("%sDistance: %10.f\n", "Banana", bananaDistance);
    printf("%sDistance: %10.f\n", "Orange", orangeDistance);
    printf("%sDistance: %10.f\n", "StarFruits", starFruitsDistance);
    printf("%sDistance: %10.f\n", "Cherry", cherryDistance);
    printf("%sDistance: %10.f\n", "WaterMelon", waterMelonDistance);
    printf("%sDistance: %10.f\n", "BokChoy", bokChoyDistance);
    printf("%sDistance: %10.f\n", "Dorian", dorianDistance);
     printf("%sDistance: %10.f\n", "Radish", radishDistance);
    printf("%sDistance: %10.f\n", "Melon", melonDistance);
    printf("%sDistance: %10.f\n", "Cucumber", cucumberDistance);
    printf("%sDistance: %10.f\n", "Strawberry", strawberryDistance);
    printf("以上");

    */

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

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
  pwm = 25;
  kp = 0.5;
  ki = 0.2;
  kd = 0.45;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
  calibrator->addPIDTracer(orangePIDTracer);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

  // StarFruitsWalkerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 16;
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
  pwm = 30;
  kp = 0.5;
  ki = 0.2;
  kd = 0.5;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
  calibrator->addPIDTracer(waterMelonPIDTracer);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

  // BokChoyWalkerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 20;
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

  // AsparagusPIDTracerの初期化とCommandExecutorへの追加
  pwm = 25;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *asparagusPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateAsparagus = new WheelDistancePredicate(asparagusDistance, robotAPI);
  calibrator->addPIDTracer(asparagusPIDTracer);
  commandExecutor->addCommand(asparagusPIDTracer, predicateAsparagus, GET_VARIABLE_NAME(asparagusPIDTracer));

  // RadishPIDTracerの初期化とCommandExecutorへの追加
  pwm = 25;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *radishPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateRadish = new WheelDistancePredicate(radishDistance, robotAPI);
  calibrator->addPIDTracer(radishPIDTracer);
  commandExecutor->addCommand(radishPIDTracer, predicateRadish, GET_VARIABLE_NAME(radishPIDTracer));

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

  pwm = 35;
  kp = 0.4;
  ki = 0.3;
  kd = 0.4;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
  calibrator->addPIDTracer(cucumberPIDTracer);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
  calibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

  // CabbagePIDTracerの初期化とCommandExecutorへの追加
  pwm = 40;
  kp = 0.4;
  ki = 0.2;
  kd = 0.4;
  dt = 1;
  PIDTracer *cabbagePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCabbage = new WheelDistancePredicate(cabbageDistance, robotAPI);
  calibrator->addPIDTracer(cabbagePIDTracer);
  commandExecutor->addCommand(cabbagePIDTracer, predicateCabbage, GET_VARIABLE_NAME(cabbagePIDTracer));

  // Commandの定義とCommandExecutorへの追加ここまで

#ifdef SimulatorMode
  // シミュレータはPIDTargetBrightnessをキャリブレーションしないので値を設定する必要がある
  bananaPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  orangePIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  cherryPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  waterMelonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  dorianPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  radishPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  melonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  cucumberPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  strawberryPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
  cabbagePIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
#endif
}
#endif

#ifdef GoalKomichiScnenarioTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

  float leftPow;
  float rightPow;
  int distance;

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

#define SonarStarter
#ifdef SonarStarter
  Walker *sonarStandby = new Walker(leftPow, rightPow);
  Predicate *sonarStater = new SonarDistancePredicate(10, true);
  commandExecutor->addCommand(sonarStandby, sonarStater, GET_VARIABLE_NAME(sonarStandby));
#endif

  pwm = 22;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  distance = 8;
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  calibrator->addPIDTracer(bananaPIDTracer);

  leftPow = 0;
  rightPow = 0;
  Walker *walkerS = new Walker(leftPow, rightPow);
  // Predicate *predicate0 = new NumberOfTimesPredicate(4);
  Predicate *predicate0 = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(bananaPIDTracer, predicate0, GET_VARIABLE_NAME(bananaPIDTracer));
  //第一直進
  leftPow = 50;
  rightPow = 50;
  walkerS = new Walker(leftPow, rightPow);
  // Predicate *predicate1 = new WheelDistancePredicate(40, robotAPI);
  Predicate *predicate1 = new WheelDistancePredicate(20, robotAPI);
  commandExecutor->addCommand(walkerS, predicate1, GET_VARIABLE_NAME(walkerS));
  //第二カーブ
  leftPow = 50;
  rightPow = 10;
  Walker *walker2 = new Walker(leftPow, rightPow);
  Predicate *predicate2 = new WheelDistancePredicate(24, robotAPI);
  commandExecutor->addCommand(walker2, predicate2, GET_VARIABLE_NAME(walker2));
  //第三直進
  leftPow = 50;
  rightPow = 50;
  Walker *walker3 = new Walker(leftPow, rightPow);
  Predicate *predicate3 = new WheelDistancePredicate(52, robotAPI);
  commandExecutor->addCommand(walkerS, predicate3, GET_VARIABLE_NAME(walkerS));
  //第四カーブ
  leftPow = 50;
  rightPow = 10;
  Walker *walker4 = new Walker(leftPow, rightPow);
  Predicate *predicate4 = new WheelDistancePredicate(25, robotAPI);
  commandExecutor->addCommand(walker4, predicate4, GET_VARIABLE_NAME(walker4));
  // 5直進
  Predicate *predicate5 = new WheelDistancePredicate(10, robotAPI);
  commandExecutor->addCommand(walkerS, predicate5, GET_VARIABLE_NAME(walkerS));
  //第6カーブ,mid1
  leftPow = 30;
  rightPow = 50;
  Walker *walker6 = new Walker(leftPow, rightPow);
  Predicate *predicate6 = new WheelDistancePredicate(70, robotAPI);
  commandExecutor->addCommand(walker6, predicate6, GET_VARIABLE_NAME(walker6));
  Predicate *predicateMid1 = new WheelDistancePredicate(9, robotAPI);
  commandExecutor->addCommand(walkerS, predicateMid1, GET_VARIABLE_NAME(walkerS));
  // OK第7　一度目交差点から丸一周
  leftPow = 50;
  rightPow = 36;
  Walker *walker7 = new Walker(leftPow, rightPow);
  Predicate *predicate7 = new WheelDistancePredicate(340, robotAPI);
  commandExecutor->addCommand(walker7, predicate7, GET_VARIABLE_NAME(walker7));
  //第8　2度目交差点から抜ける
  leftPow = 20;
  rightPow = 50;
  Walker *walker8 = new Walker(leftPow, rightPow);
  Predicate *predicate8 = new WheelDistancePredicate(8, robotAPI);
  commandExecutor->addCommand(walker8, predicate8, GET_VARIABLE_NAME(walker8));
  // 9直進
  Predicate *predicate9 = new WheelDistancePredicate(42, robotAPI);
  commandExecutor->addCommand(walkerS, predicate9, GET_VARIABLE_NAME(walkerS));
  // 10カーブ
  leftPow = 8;
  rightPow = 50;
  Walker *walker10 = new Walker(leftPow, rightPow);
  Predicate *predicate10 = new WheelDistancePredicate(5, robotAPI);
  commandExecutor->addCommand(walker8, predicate10, GET_VARIABLE_NAME(walker8));
  // 11直進
  Predicate *predicate11 = new WheelDistancePredicate(65, robotAPI);
  commandExecutor->addCommand(walkerS, predicate11, GET_VARIABLE_NAME(walkerS));
  // 12Uカーブ
  leftPow = 50;
  rightPow = 12;
  Walker *walker12 = new Walker(leftPow, rightPow);
  Predicate *predicate12 = new WheelDistancePredicate(26, robotAPI);
  commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
  predicate12 = new WheelDistancePredicate(25, robotAPI);
  Predicate *predicateS12 = new WheelDistancePredicate(27, robotAPI);
  commandExecutor->addCommand(walkerS, predicateS12, GET_VARIABLE_NAME(walkerS));
  commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
  commandExecutor->addCommand(walkerS, predicate11, GET_VARIABLE_NAME(walkerS));
  // IF１３コースに沿って直進
  Predicate *predicate13S = new WheelDistancePredicate(155, robotAPI);
  commandExecutor->addCommand(walkerS, predicate13S, GET_VARIABLE_NAME(walkerS));
  leftPow = 13;
  rightPow = 50;
  Walker *walker13 = new Walker(leftPow, rightPow);
  Predicate *predicate13 = new WheelDistancePredicate(7, robotAPI);
  commandExecutor->addCommand(walker13, predicate13, GET_VARIABLE_NAME(walker13));
  Predicate *predicate13S2 = new WheelDistancePredicate(20, robotAPI);
  commandExecutor->addCommand(walkerS, predicate13S2, GET_VARIABLE_NAME(walkerS));
  //   //13ゴールに向かって直進
  //  leftPow = 31;
  // rightPow = 100;
  // Walker *walker13 = new Walker(leftPow, rightPow);
  // Predicate *predicate13 = new WheelDistancePredicate(2, robotAPI);
  // commandExecutor->addCommand(walker13, predicate13, GET_VARIABLE_NAME(walker13));
  // Predicate *predicate13S = new WheelDistancePredicate(151, robotAPI);
  // commandExecutor->addCommand(walkerS, predicate13S, GET_VARIABLE_NAME(walkerS));
  // Predicate *predicate13t = new WheelDistancePredicate(36, robotAPI);
  // commandExecutor->addCommand(walker13, predicate13t, GET_VARIABLE_NAME(walker13));
  // 14黒キャッチしてから　ライントレース
  /*
  leftPow = 20;
  rightPow = 20;
  Walker *walker14S = new Walker(leftPow, rightPow);
  Predicate *predicate14c = new ColorPredicate(COLOR_BLACK);
  commandExecutor->addCommand(walker14S, predicate14c, GET_VARIABLE_NAME(walker14S));
  */

  pwm = 10;
  float angle = -45;
  FacingAngleAbs *facingAngle45 = new FacingAngleAbs(FA_Gyro, pwm, angle);
  commandExecutor->addCommand(facingAngle45, new FinishedCommandPredicate(facingAngle45), GET_VARIABLE_NAME(facingAngle45));

  leftPow = 20;
  rightPow = 20;
  Walker *lowWalker = new Walker(leftPow, rightPow);
  Predicate *blackPredicate = new BlackPredicate();
  commandExecutor->addCommand(lowWalker, blackPredicate, GET_VARIABLE_NAME(lowWalker));

  pwm = 15;
  kp = 0.2;
  ki = 0.1;
  kd = 0.2;
  dt = 1;
  ColorPIDTracer *colorPIDTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  calibrator->addColorPIDTracer(colorPIDTracer);
  commandExecutor->addCommand(colorPIDTracer, new BlueEdgePredicate(), GET_VARIABLE_NAME(colorPIDTracer));
}
#endif

#ifdef GoalOkiharaPIDMode3
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
  //下記コメントアウト箇所アンパイ

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
  //アンパイ

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

  //下記はアンパイ
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

#ifdef GoalOkiharaPIDMode4
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // wheelDiameter = 10.5; // これは実方機体
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
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

  {
    // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
    // そのシーンが終了する距離の定義。
    // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
    // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
    /*
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
    */

    int sceneCarrotMotorCountPredicateArg = 750;      // スタートから最初のカーブ終わるまで
    int sceneBananaMotorCountPredicateArg = 1100;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
    int scenePeachMotorCountPredicateArg = 1540;      // バナナとオレンジの間の小さいカーブ
    int sceneOrangeMotorCountPredicateArg = 2450;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
    int sceneStarFruitsMotorCountPredicateArg = 2540; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
    int sceneCherryMotorCountPredicateArg = 2750;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
    int sceneBokChoyMotorCountPredicateArg = 6325;    // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
    int sceneDorianMotorCountPredicateArg = 6650;     // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneAsparagusMotorCountPredicateArg = 7100;  // ドリアン終了後の１つ目の直線
    int sceneRadishMotorCountPredicateArg = 7390;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    int sceneMelonMotorCountPredicateArg = 7840;      // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
    int sceneLemonMotorCountPredicateArg = 8690;
    int sceneCucumberMotorCountPredicateArg = 10505;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
    int sceneStrawberryMotorCountPredicateArg = 11300; // ゴールまで。いちご好き。ライントレースする。
    int sceneCabbageMotorCountpredicateArg = 12000;    // ゴールまで。

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

    /*
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
      */

    float pwm;
    float kp;
    float ki;
    float kd;
    int dt;
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
    //下記コメントアウト箇所アンパイ

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
    //アンパイ

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
    kp = 1.1;
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

    //下記はアンパイ
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
    kp = 1.0;
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

    pwm = 40;
    kp = 1.0;
    ki = 0;
    kd = 4.0;
    dt = 1;
    r = 0;

    /*
    pwm = 30;
    kp = 0.75;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;
    */

    PIDTracerV2 *strawberryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
    calibrator->addPIDTracer(strawberryPIDTracer);
    commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

    // CabbagePIDTracerの初期化とCommandExecutorへの追加

    pwm = 60;
    kp = 1.0;
    ki = 0;
    kd = 2.0;
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
}
#endif

#ifdef GoalOkiharaPIDMode3Distance
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
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

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
  //下記コメントアウト箇所アンパイ

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
  //アンパイ

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

  //下記はアンパイ
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

#ifdef GoalSanekataPIDMode2
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
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

  {
    int orangePlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    // int orangePlan = 2; // 弱めのPDでRに頼った走行
    // int orangePlan = 3; // Iに頼った走行
    // int cherryPlan = 1; // 強めのPD、なるべく弱めのRな走行
    // int cherryPlan = 2; // 弱めのPDでRに頼った走行
    int cherryPlan = 3; // まあまあのPDでRに頼った走行
    // int cherryPlan = 4; // Iに頼った走行
    // int waterMelonPlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    // int waterMelonPlan = 2; // 弱めのPDでRに頼った走行
    // int waterMelonPlan = 3; // まあまあのPDでRに頼った走行
    // int waterMelonPlan = 4; // 弱めのPIDでRに頼った走行
    int waterMelonPlan = 5; // Iに頼った走行
    // int dorianPlan = 1; // まぁまぁなPID走行 iを使うと安定性が下がるのでplan2, plan3を用意しました
    int dorianPlan = 2; // 弱めのPD走行
    // int dorianPlan = 3; // 強めのPD走行
    SetPWMCoefficient *setPWMCoefficient = new SetPWMCoefficient();
    // commandExecutor->addCommand(setPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(setPWMCoefficient));

    bool facingAngleAtStarFruits = false;
    bool facingAngleAtBokChoy = false;

    bool useAnglePredicateAtWaterMelon = false;
    bool useAnglePredicateAtOrange = false;

    float pmanDistance = 34;
    float carrotDistance = 31;
    float bananaDistance = 38;
    float peachDistance = 34;
    float orangeDistance = 72;
    float starFruitsDistance = 12;
    float cherryDistance = 60;
    float waterMelonDistance = 273.5;
    float bokChoyDistance = 15;
    float dorianDistance = 40;
    float hassakuDistance = 35;
    float radishDistance = 34;
    float melonDistance = 36;
    float nutsDistance = 10;
    float lemonDistance = 37;
    float cucumberDistance = 189;
    float strawberryDistance = 45;
    float cabbageDistance = 100;

    /*
    #ifdef Right // 学校のコース伸びた説
    orangeDistance += 1.5;
    waterMelonDistance += 5;
    #endif
    */

    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;
    float r;
    float radius;
    float theta;

    uint64_t waitFaUsec = 500000;

    FacingAngleMode facingAngleMode = FA_WheelCount;
    float angle;
    float faKp = 0.7;
    float faKi = 0;
    float faKd = 0.7;
    float faDt = 1;

    float carrotPWM = 60;
    float carrotKp = 0.7;   // 0.6;//TODO kp高めれば行けそう
    float carrotKi = 0.015; // 0.12;
    float carrotKd = carrotKp * 3;
    float carrotDt = 0.4;
    float carrotR = 39;

    // TODO これやりたい
    // carrotPWM = 65;
    // carrotKp = 0.6;
    // carrotKi = 0;//2.65;
    // carrotKd = 0.21;
    // carrotDt = 0.05;
    // carrotR = 0;//21;

    // TODO これやりたい
    carrotPWM = 60;
    carrotKp = 0.7;
    carrotKi = 0; // 2.65;
    carrotKd = 0.23;
    carrotDt = 0.05;
    carrotR = 21;

    // PmanPIDTracerの初期化とCommandExecutorへの追加
    pwm = 25;
    kp = 0.5;
    ki = 0;
    kd = 1.4;
    dt = 1;
    r = 0;
    PIDTracerV2 *pmanPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatePman = new WheelDistancePredicate(pmanDistance, robotAPI);
    commandExecutor->addCommand(pmanPIDTracer, predicatePman, GET_VARIABLE_NAME(pmanPIDTracer));
    calibrator->addPIDTracer(pmanPIDTracer);

    // CarrotPIDTracerの初期化とCommandExecutorへの追加
    /*
    pwm = 50;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = 30;
    */
    // 距離依存をへらすために速度を落としてAndPredicateを使います
    pwm = 25;
    kp = 0.8;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 12.5;
    angle = 90 - 20;
    PIDTracerV2 *carrotPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    // Predicate *predicateCarrot = new WheelDistancePredicate(carrotDistance, robotAPI);
    Predicate *predicateCarrot = new ANDPredicate(new WheelDistancePredicate(carrotDistance, robotAPI), new FacingRobotUseWheelPredicate(angle));
    commandExecutor->addCommand(carrotPIDTracer, predicateCarrot, GET_VARIABLE_NAME(carrotPIDTracer));
    calibrator->addPIDTracer(carrotPIDTracer);

    // BananaPIDTracerの初期化とCommandExecutorへの追加
    pwm = 45;
    kp = 0.44;
    ki = 0;
    kd = 1.5;
    dt = 1;
    r = 0;
    PIDTracerV2 *bananaPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
    commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
    calibrator->addPIDTracer(bananaPIDTracer);

    // PeachPIDTracerの初期化とCommandExecutorへの追加
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = carrotR;
    PIDTracerV2 *peachPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatePeach = new WheelDistancePredicate(peachDistance, robotAPI);
    calibrator->addPIDTracer(peachPIDTracer);
    commandExecutor->addCommand(peachPIDTracer, predicatePeach, GET_VARIABLE_NAME(peachPIDTracer));

    // OrangePIDTracerの初期化とCommandExecutorへの追加
    switch (orangePlan)
    {
    case 1:
    {
      // 強めのPDとI、なるべく弱めのRな走行
      orangeDistance = 72.5;
      pwm = 60;
      kp = 0.675;
      ki = 0.01;
      kd = kp * 3;
      dt = 1;
      r = -34;
      break;
    }
    case 2:
    {
      // TODO
      // 弱めのPDでRに頼った走行
      orangeDistance = 73.5; // TODO
      pwm = 60;
      kp = 0.4; // TODO
      ki = 0;
      kd = kp * 3;
      dt = 1;
      r = -38;
      break;
    }
    case 3:
    {
      pwm = 65;
      kp = 0.6;
      ki = 2.65;
      kd = 0.21;
      dt = 0.05;
      r = -20.5;
    }
    }
    angle = 0; // TODO
    PIDTracerV2 *orangePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateOrange;
    if (useAnglePredicateAtOrange)
    {
      predicateOrange = new FacingRobotUseWheelPredicate(angle);
    }
    else
    {
      predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
    }
    calibrator->addPIDTracer(orangePIDTracer);
    commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

    if (facingAngleAtStarFruits)
    {
      angle = 10;
      PIDFacingAngleAbs *facingAngleC = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleC), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleC, facingAngleCPredicate, GET_VARIABLE_NAME(facingAngleC));
    }

    // StarFruitsWalkerの初期化とCommandExecutorへの追加
    pwm = 60;
    radius = 17;
    theta = -360; // 多めにしないと動かんのか？
    angle = 24;
    CurvatureWalkerCommandAndPredicate *starFuitsWalker = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
    Predicate *predicateStarFruits = new WheelDistancePredicate(starFruitsDistance, robotAPI);
    // Predicate *predicateStarFruits = new FacingRobotUseWheelPredicate(angle);
    // Predicate *predicateStarFruits = new GyroRotateAnglePredicate(angle);
    predicateStarFruits = predicateStarFruits->generateReversePredicate(); // 右車輪のほうが回転数多くなるのでそちらではかったほうが精度高くなりそう
    commandExecutor->addCommand(starFuitsWalker->getCommand(), predicateStarFruits, GET_VARIABLE_NAME(starFuitsWalker));
    float cherryPWM;
    float cherryKp;
    float cherryKi;
    float cherryKd;
    float cherryDt;
    float cherryR;
    float waterMelonPWM;
    float waterMelonKp;
    float waterMelonKi;
    float waterMelonKd;
    float waterMelonDt;
    float waterMelonR;

    switch (cherryPlan)
    {
    case 1:
    {
      // 強めのPD、なるべく弱めのRな走行
      cherryDistance = 60;
      cherryPWM = 65;
      cherryKp = 0.7;
      cherryKi = 0; // 0.025;
      cherryKd = 2.1;
      cherryDt = 1;
      cherryR = 38; // TODO
      break;
    }
    case 2:
    {
      // 弱めのPDでRに頼った走行
      cherryDistance = 70; // TODO
      cherryPWM = 65;
      cherryKp = 0.38; // TODO
      cherryKi = 0;
      cherryKd = cherryKp * 3; // TODO
      cherryDt = 1;
      cherryR = 31; // TODO
      break;
    }
    case 3:
    {
      // TODO
      cherryDistance = 60;
      cherryPWM = 65;
      cherryKp = 0.5;
      cherryKi = 0;
      cherryKd = cherryKp * 3;
      cherryDt = 1;
      cherryR = 28; // TODO
      break;
    }
    case 4:
    {
      // Iに頼った走行
      cherryDistance = 60;
      cherryPWM = 65;
      cherryKp = 0.6;
      cherryKi = 2.65;
      cherryKd = 0.21;
      cherryDt = 0.05;
      cherryR = 25;
      break;
    }
    }

    switch (waterMelonPlan)
    {
    case 1:
    {
      // 強めのPDとI、なるべく弱めのRな走行
      waterMelonDistance = 273.5;
      waterMelonPWM = 65;
      waterMelonKp = 0.4; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 45; // TODO
      break;
    }
    case 2:
    {
      // 弱めのPDでRに頼った走行
      waterMelonDistance = 261;
      waterMelonPWM = 65;
      waterMelonKp = 0.4; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 33; // TODO
      break;
    }
    case 3:
    {
      // まあまあのPDでRに頼った走行
      waterMelonDistance = 264;
      waterMelonPWM = 65;
      waterMelonKp = 0.5; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 33; // TODO
      break;
    }
    case 4:
    {
      // 弱めのPIDでRに頼った走行
      waterMelonDistance = 253;
      waterMelonPWM = 65;
      waterMelonKp = 0.4;
      waterMelonKi = 0.01;
      waterMelonKd = waterMelonKp * 3;
      waterMelonDt = 1;
      waterMelonR = 29; // TODO
      break;
    }
    case 5:
      // Iに頼った走行
      {
        waterMelonPWM = 65;
        waterMelonKp = 0.6;
        waterMelonKi = 2.65;
        waterMelonKd = 0.21;
        waterMelonDt = 0.05;
        waterMelonR = 25;
        break;
      }
    }

    pwm = cherryPWM;
    kp = cherryKp;
    ki = cherryKi;
    kd = cherryKd;
    dt = cherryDt;
    r = cherryR;
    PIDTracerV2 *cherryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCherry = new WheelDistancePredicate(cherryDistance, robotAPI);
    calibrator->addPIDTracer(cherryPIDTracer);
    commandExecutor->addCommand(cherryPIDTracer, predicateCherry, GET_VARIABLE_NAME(cherryPIDTracer));

    // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
    pwm = waterMelonPWM;
    kp = waterMelonKp;
    ki = waterMelonKi;
    kd = waterMelonKd;
    dt = waterMelonDt;
    r = waterMelonR;
    angle = 0; // TODO
    PIDTracerV2 *waterMelonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateWaterMelon;
    if (useAnglePredicateAtWaterMelon)
    {
      predicateWaterMelon = new FacingRobotUseWheelPredicate(angle);
    }
    else
    {
      predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
    }
    calibrator->addPIDTracer(waterMelonPIDTracer);
    commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

    if (facingAngleAtBokChoy)
    {
      angle = 330;
      PIDFacingAngleAbs *facingAngleC = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleC), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleC, facingAngleCPredicate, GET_VARIABLE_NAME(facingAngleC));
    }

    // BokChoyWalkerの初期化とCommandExecutorへの追加
    pwm = 60;
    radius = 23.5;
    theta = -360; // 多めにしないと動かんのか？
    CurvatureWalkerCommandAndPredicate *bokChoyWalker = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
    Predicate *predicateBokChoy = new WheelDistancePredicate(bokChoyDistance, robotAPI);
    predicateBokChoy = predicateBokChoy->generateReversePredicate(); // 右車輪のほうが回転数多くなるのでそちらではかったほうが精度高くなりそう
    commandExecutor->addCommand(bokChoyWalker->getCommand(), predicateBokChoy, GET_VARIABLE_NAME(bokChoyWalker));

    float dorianPWM;
    float dorianKp;
    float dorianKi;
    float dorianKd;
    float dorianDt;
    float dorianR;

    // DorianPIDTracerの初期化とCommandExecutorへの追加
    switch (dorianPlan)
    {
    case 1:
    {
      // まぁまぁなPID走行
      dorianPWM = 35;
      dorianKp = 0.58;
      dorianKi = 0.006;
      dorianKd = kp * 3;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    case 2:
    {
      // 弱めのPD走行
      dorianPWM = 35;
      dorianKp = 0.48;
      dorianKi = 0;
      dorianKd = kp * 3;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    case 3:
    {
      // 強めのPD走行
      dorianPWM = 35;
      dorianKp = 0.5;
      dorianKi = 0;
      dorianKd = kp * 3;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    case 4:
    {
      // ばななを使う
      dorianPWM = 45;
      dorianKp = 0.44;
      dorianKi = 0;
      dorianKd = 1.5;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    }
    pwm = dorianPWM;
    kp = dorianKp;
    ki = dorianKi;
    kd = dorianKd;
    dt = dorianDt;
    r = dorianR;
    PIDTracerV2 *dorianPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateDorian = new WheelDistancePredicate(dorianDistance, robotAPI);
    calibrator->addPIDTracer(dorianPIDTracer);
    commandExecutor->addCommand(dorianPIDTracer, predicateDorian, GET_VARIABLE_NAME(dorianPIDTracer));

    // HassakuPIDTracerの初期化とCommandExecutorへの追加
    pwm = 25;
    kp = 0.64;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *hassakuPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateHassaku = new WheelDistancePredicate(hassakuDistance, robotAPI);
    calibrator->addPIDTracer(hassakuPIDTracer);
    commandExecutor->addCommand(hassakuPIDTracer, predicateHassaku, GET_VARIABLE_NAME(hassakuPIDTracer));

    // RadishPIDTracerの初期化とCommandExecutorへの追加
    pwm = dorianPWM;
    kp = dorianKp;
    ki = dorianKi;
    kd = dorianKd;
    dt = dorianDt;
    r = dorianR;
    PIDTracerV2 *radishPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateRadish = new WheelDistancePredicate(radishDistance, robotAPI);
    calibrator->addPIDTracer(radishPIDTracer);
    commandExecutor->addCommand(radishPIDTracer, predicateRadish, GET_VARIABLE_NAME(radishPIDTracer));

    // MelonPIDTracerの初期化とCommandExecutorへの追加  ここから
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = carrotR;
    PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
    calibrator->addPIDTracer(melonPIDTracer);
    commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

    // nutsの初期化とCommandExecutorへの追加  ここから
    pwm = 50;
    kp = 0.44;
    ki = 0.001;
    kd = 1.5;
    dt = 1;
    r = 0;
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *nutsPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatenuts = new WheelDistancePredicate(nutsDistance, robotAPI);
    calibrator->addPIDTracer(nutsPIDTracer);
    commandExecutor->addCommand(nutsPIDTracer, predicatenuts, GET_VARIABLE_NAME(nutsPIDTracer));

    // LemonPIDTracerの初期化とCommandExecutorへの追加
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = carrotR;

    PIDTracerV2 *lemonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateLemon = new WheelDistancePredicate(lemonDistance, robotAPI);
    calibrator->addPIDTracer(lemonPIDTracer);
    commandExecutor->addCommand(lemonPIDTracer, predicateLemon, GET_VARIABLE_NAME(lemonPIDTracer));

    // CucumberPIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.6;
    ki = 2.65;
    kd = 0.21;
    dt = 0.05;
    r = 5;

    PIDTracerV2 *cucumberPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
    calibrator->addPIDTracer(cucumberPIDTracer);
    commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

    // StrawberryPIDTracerの初期化とCommandExecutorへの追加
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = kp; // TODO 試して
    dt = carrotDt;
    r = -44;
    PIDTracerV2 *strawberryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
    predicateStrawberry = predicateStrawberry->generateReversePredicate();
    calibrator->addPIDTracer(strawberryPIDTracer);
    commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

    // CabbagePIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.5;
    ki = 0.001;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *cabbagePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCabbage = new WheelDistancePredicate(cabbageDistance, robotAPI);
    calibrator->addPIDTracer(cabbagePIDTracer);
    commandExecutor->addCommand(cabbagePIDTracer, predicateCabbage, GET_VARIABLE_NAME(cabbagePIDTracer));
    // Commandの定義とCommandExecutorへの追加ここまで

    ResetPWMCoefficient *resetPWMCoefficient = new ResetPWMCoefficient();
    commandExecutor->addCommand(resetPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetPWMCoefficient));

#if defined(SimulatorMode) | defined(DisableCalibration)
    // シミュレータはPIDTargetBrightnessをキャリブレーションしないので値を設定する必要がある
    carrotPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    bananaPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    peachPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    orangePIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    waterMelonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    dorianPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    radishPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    melonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    lemonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    cucumberPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    strawberryPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    cabbagePIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
#endif
  }
}
#endif

#ifdef SlalomUFOTestMode
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

#ifdef SlalomAwaitingSignalPlan1SanekataMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float slalomAngle = 0; // 多分270
  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();

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

  bool facingAngleUseGyro = false;

  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);

  // Calibrator
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

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
  // commandExecutor->addCommand(pidTracer, pidTracerPredicate, GET_VARIABLE_NAME(pidTracer));

  // スラローム直前までPIDトレース
  float distance = 30;
  // commandExecutor->addCommand(pidTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
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
#ifdef SimulatorMode
  pwm = 25;
#else
  pwm = 100;
#endif
  numberOfTime = 20;
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
  leftPWM = 5;
  rightPWM = 5;
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
#ifdef SimulatorMode
  pwm = 25;
#else
  pwm = 100;
#endif
  numberOfTime = 10;
  tailMotorDrive = new TailController(-pwm);
  tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  leftPWM = 10;
  rightPWM = 10;
  distance = 25;
  Walker *walkerC = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerCPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerC, walkerCPredicate, GET_VARIABLE_NAME(walkerC));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // テールモータを戻す
#ifdef SimulatorMode
  pwm = 25;
#else
  pwm = 100;
#endif
  numberOfTime = 10;
  tailMotorDrive = new TailController(-pwm);
  tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
  commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 角度を調整する
  pwm = 5;
  FacingAngleAbs *facingAngle = new FacingAngleAbs(FacingAngleMode, pwm, slalomAngle, facingAngleUseGyro);
  commandExecutor->addCommand(facingAngle, new FinishedCommandPredicate(facingAngle), GET_VARIABLE_NAME(FacingAngle));

  // スラローム進入ここまで

  // 指示待ち走行ここから

  // 45度左旋回
  pwm = 5;
  angle = -45;
  RotateRobotUseGyroCommandAndPredicate *rotate1 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate1->getCommand(), rotate1->getPredicate(), GET_VARIABLE_NAME(rotate1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10;
  rightPWM = 10;
  distance = 11.5;
  Walker *walker2 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker2Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker2, walker2Predicate, GET_VARIABLE_NAME(walker2));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 45度右旋回
  pwm = 5;
  angle = 45;
  RotateRobotUseGyroCommandAndPredicate *rotate2 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotate2->getCommand(), rotate2->getPredicate(), GET_VARIABLE_NAME(rotate2));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 角度を調整する
  pwm = 7;
  FacingAngleAbs *facingAngleAbs = new FacingAngleAbs(pwm, slalomAngle, facingAngleUseGyro);
  commandExecutor->addCommand(facingAngle, new FinishedCommandPredicate(facingAngle), GET_VARIABLE_NAME(FacingAngle));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 10;
  rightPWM = 10;
#endif
  distance = 20;
  Walker *walker3 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker3Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker3, walker3Predicate, GET_VARIABLE_NAME(walker3));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 7;
#endif
  r = 18;
  theta = 30;
  CurvatureWalkerCommandAndPredicate *curve1 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve1->getCommand(), curve1->getPredicate(), GET_VARIABLE_NAME(curve1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 11;
  rightPWM = 11;
#endif
  distance = 8;
  Walker *walker4 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker4Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker4, walker4Predicate, GET_VARIABLE_NAME(walker4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 角度を調整する
  pwm = 7;
  facingAngleAbs = new FacingAngleAbs(pwm, slalomAngle, facingAngleUseGyro);
  commandExecutor->addCommand(facingAngle, new FinishedCommandPredicate(facingAngle), GET_VARIABLE_NAME(FacingAngle));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 14;
  Walker *walkerB = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerBPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerB, walkerBPredicate, GET_VARIABLE_NAME(walkerB));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 45度左旋回
#ifdef SimulatorMode
  pwm = 10;
#else
  pwm = 6;
#endif
  angle = -45;
  RotateRobotUseGyroCommandAndPredicate *rotateA = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotateA->getCommand(), rotateA->getPredicate(), GET_VARIABLE_NAME(rotateA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 0.5;
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
  theta = 50;
  CurvatureWalkerCommandAndPredicate *curve2 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve2->getCommand(), curve2->getPredicate(), GET_VARIABLE_NAME(curve2));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 5;
  Walker *walkerQ = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerQPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerQ, walkerQPredicate, GET_VARIABLE_NAME(walkerQ));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 8;
#endif
  r = 8;
  theta = 50;
  CurvatureWalkerCommandAndPredicate *curve3 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve3->getCommand(), curve3->getPredicate(), GET_VARIABLE_NAME(curve3));

  //  直進
#ifdef SimulatorMode
  leftPWM = 15;
  rightPWM = 15;
#else
  leftPWM = 7;
  rightPWM = 7;
#endif
  distance = 10;
  Walker *walker6 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker6Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker6, walker6Predicate, GET_VARIABLE_NAME(walker6));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 10;
#endif
  r = 10;
  theta = -45;
  CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
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
  CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
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
  theta = 40;
  CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));

  // カーブ
#ifdef SimulatorMode
  pwm = 20;
#else
  pwm = 7;
#endif
  r = 10;
  theta = -65;
  CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));

  // PIDで青線まで進む
  commandExecutor->addCommand(lowPWMTracer, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(lowPWMTracer));

  // 指示待ち走行ここまで
#endif

  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#ifdef SlalomAwaitingSignalPlan2SanekataMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

  float leftPWM;
  float rightPWM;

  int numberOfTime;

  FacingAngleMode facingAngleMode = FA_WheelCount;

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "Stopper");

  // ↓ここから実方↓
  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "Stopper");

  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

  float slalomAngleOffset = 0;

  float coefficientPWM;
  float coefficientPWMForFacingAngle;
  float coefficientPWMForCurve;

  float radius;
  float theta;

  float angle;
  float distance;

#ifdef SimulatorMode
  coefficientPWM = 2;
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
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
#else
  kp = 0.19;
  ki = 0.15;
  kd = 0.19;
  dt = 1;
#endif
  pwm = 15 * coefficientPWM;
  ColorPIDTracer *pidTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 10 * coefficientPWM;
  ColorPIDTracer *lowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 5 * coefficientPWM;
  ColorPIDTracer *verryLowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  calibrator->addColorPIDTracer(pidTracer);
  calibrator->addColorPIDTracer(lowPWMTracer);
  calibrator->addColorPIDTracer(verryLowPWMTracer);
#ifdef SimulatorMode
  float targetBrightness = 20;
  rgb_raw_t targetRGB;
  targetRGB.r = blackWhiteEdgeR;
  targetRGB.g = 60;
  targetRGB.b = 60;
  pidTracer->setTargetColor(targetRGB);
  lowPWMTracer->setTargetColor(targetRGB);
#endif
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngleAtSlalom = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngleAtSlalom, new FinishedCommandPredicate(resetArmAngleAtSlalom), GET_VARIABLE_NAME(resetArmAngleAtSlalom));

  // 1.5秒止める。BrightnessからColorへの切り替えのために。
  commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));
  // uint64_t waitDurationUsec = 1000 * 1000;
  // commandExecutor->addCommand(stopper, new TimerPredicate(waitDurationUsec), "wait switch mode brightness to row color");

  // PIDトレースで青線まで進む
  Predicate *distancePredicate = new WheelDistancePredicate(40, robotAPI);
  // commandExecutor->addCommand(pidTracer, distancePredicate, GET_VARIABLE_NAME(lowPWMTracer));

  // PIDトレースで青線まで進む
  Predicate *pidTracerPredicate = new BlueEdgePredicate();
  // commandExecutor->addCommand(pidTracer, pidTracerPredicate, GET_VARIABLE_NAME(lowPWMTracer));

  // スラローム直前までPIDトレース
  distance = 26;
  // commandExecutor->addCommand(lowPWMTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
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
  numberOfTime = 80;
  leftPWM = 7 * coefficientPWM;
  rightPWM = 7 * coefficientPWM;
  Command *lowWalker = new Walker(leftPWM, rightPWM);
  commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
  commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

  // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
  leftPWM = -5 * coefficientPWM;
  rightPWM = -5 * coefficientPWM;
  distance = -3;
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

  // ジャイロセンサをリセットする
  ResetGyroSensor *resetGyroSensor = new ResetGyroSensor();
  commandExecutor->addCommand(resetGyroSensor, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetGyroSensor));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // MeasAngleをリセットする
  ResetMeasAngle *resetMeasAngle = new ResetMeasAngle();
  commandExecutor->addCommand(resetMeasAngle, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetMeasAngle));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  distance = 27;
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
  leftPWM = -7 * coefficientPWM;
  rightPWM = -7 * coefficientPWM;
  // distance = -5.2;
  distance = -3.8; // 大会で変更
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

  // スラローム位置補正ここまで

  // 指示待ち走行ここから

  // 向き調節
  pwm = 8 * coefficientPWMForFacingAngle;
  FacingAngleAbs *facingAngle1 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset);
  commandExecutor->addCommand(facingAngle1, new FinishedCommandPredicate(facingAngle1), GET_VARIABLE_NAME(facingAngle1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 位置調節
  pwm = 12 * coefficientPWM;
  distance = 7;
  Hedgehog *headgehogA = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehogA, new FinishedCommandPredicate(headgehogA), GET_VARIABLE_NAME(headgehogA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 12 * coefficientPWMForCurve;
  radius = 16;
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
  radius = 18;
  theta = -30;
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
  distance = 8;
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
  distance = 11;
  Walker *walkerD = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerDPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerD, walkerDPredicate, GET_VARIABLE_NAME(walkerD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -35;
  FacingAngleAbs *facingAngle4 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle4, new FinishedCommandPredicate(facingAngle4), GET_VARIABLE_NAME(facingAngle4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 7 * coefficientPWMForCurve;
  radius = 10;
  theta = 24;
  CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // バック
  leftPWM = -7 * coefficientPWM;
  rightPWM = -7 * coefficientPWM;
  distance = -2;
  Walker *walkerZ = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerZPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerZ, walkerZPredicate, GET_VARIABLE_NAME(walkerZ));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 14;
  theta = 55;
  CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 3;
  Walker *walker5 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 20;
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
  pwm = 12 * coefficientPWM;
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
  radius = 12;
  theta = -40;
  CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 120度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -140;
  FacingAngleAbs *facingAngle7 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle7, new FinishedCommandPredicate(facingAngle7), GET_VARIABLE_NAME(facingAngle7));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 10;
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
  distance = 20;
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
}
#endif

#ifdef SlalomAwaitingSignalPlan3SanekataMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();

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
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);

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
  float targetBrightness = 20;
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

#ifdef SlalomAwaitingSignalPlan4SanekataMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案3
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
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

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
  float targetBrightness = 20;
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

#ifdef SlalomAwaitingSignalPlan5SanekataMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案3
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
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

  {
    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;
    float pidR;
    float straightKp = 0.05;
    float straightKi = 0;
    float straightKd = 0.05;
    float straightDt = 1;
    float faKp = 0.7;
    float faKi = 0.01;
    float faKd = 0.7;
    float faDt = 1;

    commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "stopper");
    resetArmAngle = new ResetArmAngle();
    commandExecutor->addCommand(resetArmAngle, new FinishedCommandPredicate(resetArmAngle), GET_VARIABLE_NAME(resetArmAngle));

    // ガレージカードの色取得用ColorReader
    float slalomAngleOffset = 0;

    float coefficientPWM;
    float coefficientPWMForCurve;

    float radius;
    float theta;

    float angle;
    float distance;

    float leftPWM;
    float rightPWM;

    int numberOfTime;
    uint64_t waitFaUsec = 1000000;

    FacingAngleMode facingAngleMode = FA_WheelCount;

#ifdef SimulatorMode
    coefficientPWM = 2;
    coefficientPWMForCurve = 2;
#else
    coefficientPWM = 1;
    coefficientPWMForCurve = 1;
#endif

    Stopper *stopper = new Stopper();

#ifdef SimulatorMode
    pwm = 30 * coefficientPWM;
    kp = 0.7;
    ki = 0.2;
    kd = 0.7;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *pidTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
    pwm = 20 * coefficientPWM;
    ColorPIDTracerV2 *lowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
    pwm = 10 * coefficientPWM;
    ColorPIDTracerV2 *verryLowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
#else
    pwm = 30 * coefficientPWM;
    kp = 0.155;
    ki = 0.001;
    kd = 0.572;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *pidTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);

    pwm = 20 * coefficientPWM;
    kp = 0.195;
    ki = 0;
    kd = 0.39;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *lowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);

    pwm = 10 * coefficientPWM;
    kp = 0.305;
    ki = 0;
    kd = 0.522;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *verryLowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
#endif
    calibrator->addColorPIDTracer(pidTracer);
    calibrator->addColorPIDTracer(lowPWMTracer);
    calibrator->addColorPIDTracer(verryLowPWMTracer);
#ifdef SimulatorMode
    float targetBrightness = 20;
    rgb_raw_t targetRGB;
    targetRGB.r = blackWhiteEdgeR;
    targetRGB.g = 60;
    targetRGB.b = 60;
    pidTracer->setTargetColor(targetRGB);
    lowPWMTracer->setTargetColor(targetRGB);
#endif

    // スラローム進入ここから
    {
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

      /*
      // スラローム位置補正。アームを下げたまま直進。
      numberOfTime = 65;
      leftPWM = 7;
      rightPWM = 7;
      Walker *lowWalker = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
      commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");
      */
      numberOfTime = 35;
      leftPWM = 12;
      rightPWM = 12;
      Walker *lowWalker0 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker0, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker0));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = 12;
      rightPWM = -4;
      Walker *lowWalker1 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker1, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker1));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = -4;
      rightPWM = 12;
      Walker *lowWalker2 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker2, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker2));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = 7;
      rightPWM = -2;
      Walker *lowWalker3 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker3, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker3));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = -2;
      rightPWM = 7;
      Walker *lowWalker4 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker4, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker4));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 35;
      leftPWM = 7;
      rightPWM = 7;
      Walker *lowWalker5 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker5, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker5));
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
      pwm = -5 * coefficientPWM;
      distance = -4;
      PIDStraightWalker *back = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
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
      distance = 27;
      pwm = 30;
      PIDStraightWalker *walker1_y = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walker1_y->setTargetDifferenceWheelCount(0);
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
    }
    // スラローム進入ここまで

    // スラローム位置補正ここから
    {
      // ジャイロで向き調節
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
      angle = -90;
      PIDFacingAngleAbs *facingAngleX = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleXPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleX), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleX, facingAngleXPredicate, GET_VARIABLE_NAME(facingAngleX));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 白を拾うまで直進
      pwm = 5;
      pwm = 7;
      PIDStraightWalker *walkerW = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walkerW->setTargetDifferenceWheelCount(0);
      RawColorPredicate *whitePredicate = new WhiteAtSlaromPredicate();
      commandExecutor->addCommand(walkerW, whitePredicate, GET_VARIABLE_NAME(walkerW));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // バック
      pwm = -6;
      pwm = -10;
      distance = -4.2;
      PIDStraightWalker *walkerB = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walkerB->setTargetDifferenceWheelCount(0);
      WheelDistancePredicate *walkerBPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walkerB, walkerBPredicate, GET_VARIABLE_NAME(walkerB));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      angle = 0;
      PIDFacingAngleAbs *facingAngleC = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleC), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleC, facingAngleCPredicate, GET_VARIABLE_NAME(facingAngleC));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      leftPWM = 8;
      rightPWM = 8;
      Walker *walkerC2 = new Walker(leftPWM, rightPWM);
      Predicate *walkerC2Predicate = new WheelDistancePredicate(6, robotAPI);
      commandExecutor->addCommand(walkerC2, walkerC2Predicate, GET_VARIABLE_NAME(walkerC2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

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

      angle = 0;
      FacingAngleAbs *facingAngleC3 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
      commandExecutor->addCommand(facingAngleC3, new FinishedCommandPredicate(facingAngleC3), GET_VARIABLE_NAME(facingAngleC3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */
    }
    // スラローム位置補正ここまで

    // 指示待ち走行ここから
    {
      // 向き調節
      angle = 0;
      PIDFacingAngleAbs *facingAngle1 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle1Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle1), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle1, facingAngle1Predicate, GET_VARIABLE_NAME(facingAngle1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 位置調節
      pwm = 8 * coefficientPWM;
      pwm = 10 * coefficientPWM;
      distance = 10;
      HedgehogUsePID *headgehogA = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehogA, new FinishedCommandPredicate(headgehogA), GET_VARIABLE_NAME(headgehogA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      pwm = 15 * coefficientPWMForCurve;
      radius = 14;
      theta = 50;
      CurvatureWalkerCommandAndPredicate *curve1 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve1->getCommand(), curve1->getPredicate(), GET_VARIABLE_NAME(curve1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 7 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      pwm = 15 * coefficientPWMForCurve;
      radius = 14;
      theta = -50;
      CurvatureWalkerCommandAndPredicate *curve2 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve2->getCommand(), curve2->getPredicate(), GET_VARIABLE_NAME(curve2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      pwm = 10 * coefficientPWMForCurve;
      angle = 0;
      PIDFacingAngleAbs *facingAngle3 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle3Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle3), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle3, facingAngle3Predicate, GET_VARIABLE_NAME(facingAngle3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 直進
      pwm = 7;
      pwm = 7;
      distance = 2;
      PIDStraightWalker *walkerA = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      WheelDistancePredicate *walkerAPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walkerA, walkerAPredicate, GET_VARIABLE_NAME(walkerA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 位置調節
      pwm = 7 * coefficientPWM;
      distance = 8;
      Hedgehog *headgehog1 = new Hedgehog(distance, pwm);
      commandExecutor->addCommand(headgehog1, new FinishedCommandPredicate(headgehog1), GET_VARIABLE_NAME(headgehog1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      radius = 12.5;
      theta = -45;
      CurvatureWalkerCommandAndPredicate *curve3 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve3->getCommand(), curve3->getPredicate(), GET_VARIABLE_NAME(curve3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      distance = 1.5;
      pwm = 10;
      PIDStraightWalker *walkerD = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      WheelDistancePredicate *walkerDPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walkerD, walkerDPredicate, GET_VARIABLE_NAME(walkerD));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      /*
      // 向き調節
      angle = -45;
      PIDFacingAngleAbs *facingAngle4 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle4Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle4), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle4, facingAngle4Predicate, GET_VARIABLE_NAME(facingAngle4));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 7 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      radius = 12.5;
      theta = 45;
      CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

/*
// 直進位置調節
pwm = -10 * coefficientPWM;
distance = -3;
HedgehogUsePID *headgehogZ = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
commandExecutor->addCommand(headgehogZ, new FinishedCommandPredicate(headgehogZ), GET_VARIABLE_NAME(headgehogZ));
commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
*/

// 直進
#ifdef SlalomPattern1
      distance = 8.5;
      pwm = 10;
      PIDStraightWalker *walker5 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
#else
      distance = 4;
      pwm = 10;
      PIDStraightWalker *walker5 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
#endif

      // 向き調節
      angle = 0;
      PIDFacingAngleAbs *facingAngle42 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle42Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle42), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle42, facingAngle42Predicate, GET_VARIABLE_NAME(facingAngle42));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 8 * coefficientPWMForCurve;
      radius = 11.2; // 11.5;
      theta = 50;
      CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 5 * coefficientPWMForCurve;
      radius = 11.2; // 11.5;
      theta = -50;
      CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      PIDFacingAngleAbs *facingAngle5 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset, faKp, faKi, faKd, faDt);
      Predicate *facingAngle5Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle5), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle5, facingAngle5Predicate, GET_VARIABLE_NAME(facingAngle5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 直進位置調節
      int diff = 2;
      // pwm = 10 * coefficientPWM;
      pwm = 7 * coefficientPWM;
      distance = 3 + diff;
      HedgehogUsePID *headgehog2 = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehog2, new FinishedCommandPredicate(headgehog2), GET_VARIABLE_NAME(headgehog2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      angle = 23.5;
      PIDFacingAngleAbs *facingAngleCo1 = new PIDFacingAngleAbs(facingAngleMode, angle + slalomAngleOffset, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCo1Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngleCo1), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleCo1, facingAngleCo1Predicate, GET_VARIABLE_NAME(facingAngleCo1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 色取得
      commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));

      // 向き調節
      PIDFacingAngleAbs *facingAngleCo2 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCo2Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngleCo2), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleCo2, facingAngleCo2Predicate, GET_VARIABLE_NAME(facingAngleCo2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

#ifdef SlalomPattern1
      // 直進位置調節
      pwm = 10 * coefficientPWM;
      pwm = 7;
      // pwm = 20 * coefficientPWM;
      distance = 12 + diff;
      HedgehogUsePID *headgehogAA = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehogAA, new FinishedCommandPredicate(headgehogAA), GET_VARIABLE_NAME(headgehogAA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      radius = 12.5;
      theta = -87.5;
      CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 90度左を向く
      angle = -90;
      PIDFacingAngleAbs *facingAngleY = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleYPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleY), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleY, facingAngleYPredicate, GET_VARIABLE_NAME(facingAngleY));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 位置調節
      pwm = 6 * coefficientPWM;
      // pwm = 10 * coefficientPWM;
      pwm = 7;
      distance = 5;
      HedgehogUsePID *headgehog3 = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehog3, new FinishedCommandPredicate(headgehog3), GET_VARIABLE_NAME(headgehog3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 直進
      distance = 2.5;
      pwm = 6;
      pwm = 10;
      PIDStraightWalker *walker7 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 150度左を向く
      angle = -150;
      PIDFacingAngleAbs *facingAngle8 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle8Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle8), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle8, facingAngle8Predicate, GET_VARIABLE_NAME(facingAngle8));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 37;
      theta = 70;
      CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 30度左を向く
      angle = -30;
      radius = 18;
      PIDFacingAngleAbs *facingAngle9 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle9Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle9), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle9, facingAngle9Predicate, GET_VARIABLE_NAME(facingAngle9));
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
      // 直進位置調節
      pwm = 10 * coefficientPWM;
      pwm = 7;
      // pwm = 20 * coefficientPWM;
      distance = 16 + diff;
      HedgehogUsePID *headgehogAA = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehogAA, new FinishedCommandPredicate(headgehogAA), GET_VARIABLE_NAME(headgehogAA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 10;
      theta = -40;
      CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 115度左を向く
      angle = -122.5;
      PIDFacingAngleAbs *facingAngle7 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle7Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle7), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle7, facingAngle7Predicate, GET_VARIABLE_NAME(facingAngle7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      distance = 5;
      pwm = 8;
      PIDStraightWalker *walker7 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walker7->setTargetDifferenceWheelCount(0);
      WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 43;
      theta = 21;
      CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 90度左を向く
      angle = -90;
      PIDFacingAngleAbs *facingAngle8 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle8Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle8), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle8, facingAngle8Predicate, GET_VARIABLE_NAME(facingAngle8));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      leftPWM = 10 * coefficientPWM;
      rightPWM = 10 * coefficientPWM;
      distance = 5;
      Walker *walker8y = new Walker(leftPWM, rightPWM);
      WheelDistancePredicate *walker8yPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker8y, walker8yPredicate, GET_VARIABLE_NAME(walker8y));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 15;
      theta = 50;
      CurvatureWalkerCommandAndPredicate *curve7 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve7->getCommand(), curve7->getPredicate(), GET_VARIABLE_NAME(curve7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 18;
      theta = -30;
      CurvatureWalkerCommandAndPredicate *curve8 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve8->getCommand(), curve8->getPredicate(), GET_VARIABLE_NAME(curve8));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      radius = 18;
      theta = -360;
      CurvatureWalkerCommandAndPredicate *curve9 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve9->getCommand(), new BlackPredicate(), GET_VARIABLE_NAME(curve9));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      distance = 15;
      pwm = 10;
      PIDStraightWalker *walker9 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walker9->setTargetDifferenceWheelCount(0);
      WheelDistancePredicate *walker9Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker9, walker9Predicate, GET_VARIABLE_NAME(walker9));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      /*
      // 110度左を向く
      angle = -150;
      PIDFacingAngleAbs *facingAngle10 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle10Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle10), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle10, facingAngle10Predicate, GET_VARIABLE_NAME(facingAngle10));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      /*
      // 黒線まで直進する
      pwm = 10;
      PIDStraightWalker *walkerO = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *blackPredicate = new BlackPredicate();
      commandExecutor->addCommand(walkerO, blackPredicate, GET_VARIABLE_NAME(walkerO));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // 青線までPIDトレースする
      RawColorPredicate *blueEdgePredicate = new BlueEdgePredicate();
      commandExecutor->addCommand(verryLowPWMTracer, blueEdgePredicate, GET_VARIABLE_NAME(lowPWMTracer));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
#endif
    }
    // 指示待ち走行ここまで
  }
}
#endif

#ifdef BlockTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // 停止コマンドの初期化とCommandExecutorへの追加
  int numberOfTimes = 1;
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(numberOfTimes);
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
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

// ColorIDReaderModeの場合のcommandExecutor初期化処理
#ifdef ColorIDReaderMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // rgbRawReaderの初期化とCommandExecutorへの追加
  ColorIDReader *reader = new ColorIDReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(reader, startButtonPredicate, GET_VARIABLE_NAME(reader));
}
#endif

#ifdef BrightnessReaderMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  BrightnessReader *reader = new BrightnessReader();
  commandExecutor->addCommand(reader, new Predicate(), GET_VARIABLE_NAME(reader));
}
#endif

// Rotate360TestModeの場合のcommandExecutor初期化処理
#if defined(Rotate360TestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float motorRotateAngle = 540; // ここの値をいじってはかって

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  float pwm = 10;
  Walker *walker = new Walker(pwm, -pwm); // 右に向く
  Predicate *walkerPredicate = new MotorCountPredicate(robotAPI->getLeftWheel(), motorRotateAngle);
  commandExecutor->addCommand(walker, walkerPredicate, GET_VARIABLE_NAME(walker));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
}
#endif

// RotateTestModeの場合のcommandExecutor初期化処理
#ifdef RotateTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  float angle = 10;
  float pwm = 15;
  RotateRobotCommandAndPredicate *commandAndPredicate = new RotateRobotCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), "rotateRobot");

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

// NOTE ジャイロ、 実機とシミュレータで左右判定が逆になる？
#ifdef RotateGyroTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  float pwm = 10;
  float angle = -30;
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

  // 直進コマンドの初期化とCommandExecutorへの追加
  float pwm = 50;
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

#ifdef CurvatureWalkerTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 曲率進行コマンドの初期化とCommandExecutorへの追加
  float pwm = 20; // NOTE pwm上げるとおかしくなる
  float r = 30;
  float theta = 90;
  CurvatureWalkerCommandAndPredicate *commandAndPredicate = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), "curvatureWalker");

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

#ifdef SwingSonarDetectorTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 障害物検出コマンドの初期化とCommandExecutorへの追加
  float pwm = 10;
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

#ifdef ShigekiTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float pwm = 10;
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

#ifdef UFORunnerSwingTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

// UFO走行コマンドの初期化とCommandExecutorへの追加
#ifdef SimulatorMode
  float n = 8;
  float walkerPWM = 20;
  float rotatePWM = 5;

  float swingLeftAngle = -90.0;
  float swingRightAngle = 90.0;

  float targetLeftDistance = 30;
  float targetRightDistance = 10;
  bool reverseTest = false;
#else
  float n = 5;
  float walkerPWM = 10;
  float rotatePWM = 4;

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

#ifdef UFORunnerClockwiseTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // UFO走行コマンドの初期化とCommandExecutorへの追加
#ifdef SimulatorMode
  float n = 5;
  float walkerPWM = 15;
  float rotatePWM = 3;

  float angle = 180;
  int targetLeftDistance = 20;  // これを検知した状態からはじめて
  int targetRightDistance = 20; // あとはSwingSonarと同じ

  int skipFrameAfterDetectFirstObstacle = 0;
  bool facingObstacle = true;
  bool reverseTest = true;
#else
  float n = 4;
  float walkerPWM = 15;
  float rotatePWM = 5;

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
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float pwm = 20;
  float kp = 0.7;
  float ki = 0.2;
  float kd = 0.7;
  float dt = 1;
  PIDTracer *pidTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  commandExecutor->addCommand(pidTracer, new Predicate(), GET_VARIABLE_NAME(pidTracer));
  calibrator->addPIDTracer(pidTracer);
}
#endif

#ifdef BrightnessPIDTracerV2TestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngle = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngle, new FinishedCommandPredicate(resetArmAngle), GET_VARIABLE_NAME(resetArmAngle));

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;
  float r;

  // pwm60 円コース。未完成
  pwm = 60;
  kp = 0.881;
  ki = 0.025;
  kd = 0.952;
  dt = 1;
  r = 0;

  // pwm40 円コース
  pwm = 40;
  kp = 0.6265;
  ki = 0.0812;
  kd = 1.9221;
  dt = 1;
  r = 0;

  // すいか爆速R制御
  pwm = 65;
  kp = 0.8;
  ki = 0;
  kd = 4.4;
  dt = 1;
  r = 20;

  // おれんじ爆速R制御
  pwm = 65;
  kp = 0.9;
  ki = 0;
  kd = 4.4;
  dt = 1;
  r = -34;

  // きゅうり爆速
  pwm = 85;
  kp = 0.8;
  ki = 0;
  kd = 4.4;
  dt = 1;
  r = 0;

  // アスパラガス
  pwm = 30;
  kp = 0.6;
  ki = 0;
  kd = 2.0;
  dt = 1;
  r = 0;

  // すいか爆速R制御i制御あり
  pwm = 65;
  kp = 0.7;
  ki = 0.3;
  kd = 2.0;
  dt = 0.45;
  r = 20;

  // きゅうり爆速i制御あり
  pwm = 75;
  kp = 0.95;
  ki = 0.3;
  kd = 2.4;
  dt = 0.4;
  r = 0;

  // もも
  pwm = 40;
  kp = 0.25;
  ki = 0.35;
  kd = 2.7;
  dt = 0.5;
  r = 17;

  // 60ぐらいの直進PID
  pwm = 60;
  kp = 0.4;
  ki = 0.12;
  kd = 1.2;
  dt = 0.4;
  r = 0;

  // おれんじ爆速R制御i制御あり
  pwm = 60;
  kp = 0.4;
  ki = 0.12;
  kd = 1.2;
  dt = 0.4;
  r = -32.5;

  // すいか爆速R制御i制御あり
  pwm = 65;
  kp = 0.5;
  ki = 0.1;
  kd = 1.5;
  dt = 0.45;
  r = 26;

  // どりあん
  pwm = 40;
  kp = 0.7;
  ki = 0.015;
  kd = 0.43;
  dt = 0.35;
  r = 0;

  // どりあん
  pwm = 40;
  kp = 0.85;
  ki = 0;
  kd = 2.25;
  dt = 1;
  r = 0;

  // どりあん
  pwm = 35;
  kp = 0.5;
  ki = 0.01;
  kd = 1.5;
  dt = 1;
  r = 0;

  // おれんじ爆速R制御i制御あり
  pwm = 60;
  kp = 0.4;
  ki = 0.12;
  kd = 0.8;
  dt = 0.4;
  r = 0; // -28; // TODO 直進できる値を探してからRを探して

  // おれんじ爆速R制御
  pwm = 60;
  kp = 0.675;
  ki = 0;
  kd = 2.3;
  dt = 1;
  r = -26;

  // すいか爆速R制御
  pwm = 65;
  kp = 0.7;
  ki = 0;
  kd = 2;
  dt = 1;
  r = 32;

  float carrotPWM = 60;
  float carrotKp = 0.35;
  float carrotKi = 0.015; // 0.12;
  float carrotKd = carrotKp * 3;
  float carrotDt = 0.4;
  float carrotR = 55;

  // いちご
  pwm = 60;
  kp = 0.615;
  ki = 0;  // 0.15;
  kd = kp; // TODO オーバーシュートしているのならばKdが高すぎた説がありえます
  dt = 1;
  r = -40;

  // きゅうり
  float defaultKi = 0.05;
  float magnificationKd = 0.05;
  pwm = 65;
  kp = 0.8;
  ki = defaultKi;
  kd = carrotKp * magnificationKd;
  dt = 0.01;
  r = 0;

  // 1252で求めた値
  pwm = 65;
  kp = 0.6;
  ki = 2;
  kd = 0.18;
  dt = 0.5;

  // i下げる
  pwm = 65;
  kp = 0.6;  // pもまぁまぁいい感じ
  ki = 2.65; // iはとてもいい感じ
  kd = 0.25; // dtが少なすぎてkdの調節がやりづらくなっている
  dt = 0.05; // dtが少なすぎてkdの調節がやりづらくなっている

  // 実機で明日（2022-11-11）試す値1
  dt = 0.5;
  kp = 0.6;
  ki = 1.65;
  kd = 0.25;

  // 実機で明日（2022-11-11）試す値3
  dt = 0.75;
  kp = 0.6;
  ki = 1.9;
  kd = 0.29;

  // 実機で明日（2022-11-11）試す値4
  dt = 0.5;
  kp = 0.35;
  ki = 2.2;
  kd = 0.29;

  // 実機で明日（2022-11-11）試す値2
  dt = 0.5;
  kp = 0.6;
  ki = 1.65;
  kd = 0.1338;

  // i下げる
  pwm = 65;
  kp = 0.6;
  ki = 2.65;
  kd = 0.25;
  dt = 0.05;

  // スイカトレースできるけどカクつく
  pwm = 65;
  kp = 0.8;
  ki = 0.95;
  kd = 0.015;
  dt = 0.05;

  pwm = 65;
  kp = 0.8;
  ki = 0.65;
  kd = 0.01;
  dt = 0.05;

  pwm = 65;
  kp = 0.4;
  ki = 1.3;
  kd = 0.011;
  dt = 0.05;

  // きゅうりいい感じの値
  pwm = 65;
  kp = 0.6;
  ki = 2.65;
  kd = 0.21;
  dt = 0.05;
  r = 0;

  // オレンジいい感じの値 //TODO
  pwm = 65;
  kp = 0.6;
  ki = 2.65;
  kd = 0.21;
  dt = 0.05;
  r = -34;

  // スイカいい感じの値 //TODO
  pwm = 65;
  kp = 0.6;
  ki = 2.65;
  kd = 0.21;
  dt = 0.05;
  r = 5;

  PIDTracerV2 *pidTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  commandExecutor->addCommand(pidTracer, new Predicate(), GET_VARIABLE_NAME(pidTracer));
  calibrator->addPIDTracer(pidTracer);
}
#endif

#ifdef ColorPIDTracerTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float pwm = 20;
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

#ifdef ColorPIDTracerV2TestMode
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

  // pwm25 円コース
  pwm = 25;
  kp = 0.1653;
  ki = 0.041;
  kd = 0.1639;
  dt = 1;
  r = 0;

  // pwm30 円コース
  pwm = 30;
  kp = 0.185;
  ki = 0.02;
  kd = 0.422;
  dt = 1;
  r = 0;

  ColorPIDTracerV2 *colorPIDTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, r);
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

#ifdef GrayPredicateTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  Command *walker = new Walker(10, 10);
  int *r = new int(25);
  int *g = new int(30);
  int *b = new int(40);
  Predicate *grayPredicate = new RawColorPredicate(r, BETWEEN15, g, BETWEEN15, b, BETWEEN15);
  commandExecutor->addCommand(walker, grayPredicate, GET_VARIABLE_NAME(walker));
}
#endif

#ifdef ColorReaderUseRawTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngleAtSlalom = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngleAtSlalom, new FinishedCommandPredicate(resetArmAngleAtSlalom), GET_VARIABLE_NAME(resetArmAngleAtSlalom));

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  ColorReaderUseRaw *colorReader = new ColorReaderUseRaw();
  commandExecutor->addCommand(colorReader, new Predicate(), GET_VARIABLE_NAME(colorReader));
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

#ifdef FacingAngleTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float pwm = 10;
  float angle = 180;
  FacingAngleAbs *facingAngle = new FacingAngleAbs(FA_WheelCount, pwm, angle);
  Predicate *facingAnglePredicate = new ORPredicate(new FinishedCommandPredicate(facingAngle), new TimerPredicate(waitFaUsec));
  commandExecutor->addCommand(facingAngle, facingAnglePredicate, GET_VARIABLE_NAME(facingAngle));
}
#endif

#ifdef WalkerTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float leftPWM = 10;
  float rightPWM = -10;
  Walker *walker = new Walker(leftPWM, rightPWM);
  commandExecutor->addCommand(walker, new Predicate(), GET_VARIABLE_NAME(walker));
}
#endif

#ifdef BatteryEaaterMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  int targetVoltage = 0;
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float leftPWM = 100;
  float rightPWM = -100;
  Walker *walker = new Walker(leftPWM, rightPWM);
  Predicate *batteryPredicate = new BatteryPredicate(targetVoltage);
  commandExecutor->addCommand(walker, batteryPredicate, GET_VARIABLE_NAME(walker));
}
#endif

#ifdef SlalomBlockJoinTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // ↓ここから実方↓
  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "Stopper");

  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();
  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

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
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
#else
  kp = 0.19;
  ki = 0.15;
  kd = 0.19;
  dt = 1;
#endif
  pwm = 15 * coefficientPWM;
  ColorPIDTracer *pidTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 10 * coefficientPWM;
  ColorPIDTracer *lowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 5 * coefficientPWM;
  ColorPIDTracer *verryLowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  calibrator->addColorPIDTracer(pidTracer);
  calibrator->addColorPIDTracer(lowPWMTracer);
  calibrator->addColorPIDTracer(verryLowPWMTracer);
#ifdef SimulatorMode
  float targetBrightness = 20;
  rgb_raw_t targetRGB;
  targetRGB.r = blackWhiteEdgeR;
  targetRGB.g = 60;
  targetRGB.b = 60;
  pidTracer->setTargetColor(targetRGB);
  lowPWMTracer->setTargetColor(targetRGB);
#endif
  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngleAtSlalom = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngleAtSlalom, new FinishedCommandPredicate(resetArmAngleAtSlalom), GET_VARIABLE_NAME(resetArmAngleAtSlalom));
  commandExecutor->addCommand(calibrator, new StartButtonPredicate(), GET_VARIABLE_NAME(calibrator));

  // 1.5秒止める。BrightnessからColorへの切り替えのために。
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));
  uint64_t waitDurationUsec = 1000 * 1000;
  commandExecutor->addCommand(stopper, new TimerPredicate(waitDurationUsec), "wait switch mode brightness to row color");

  // PIDトレースで青線まで進む
  Predicate *distancePredicate = new WheelDistancePredicate(40, robotAPI);
  commandExecutor->addCommand(pidTracer, distancePredicate, GET_VARIABLE_NAME(lowPWMTracer));

  // PIDトレースで青線まで進む
  Predicate *pidTracerPredicate = new BlueEdgePredicate();
  commandExecutor->addCommand(pidTracer, pidTracerPredicate, GET_VARIABLE_NAME(lowPWMTracer));

  // スラローム直前までPIDトレース
  distance = 26;
  commandExecutor->addCommand(lowPWMTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // アームを下げる
  int armAngle = 15;
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
  numberOfTime = 80;
  leftPWM = 7 * coefficientPWM;
  rightPWM = 7 * coefficientPWM;
  Command *lowWalker = new Walker(leftPWM, rightPWM);
  commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
  commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(5), "releaseWheel");

  // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
  leftPWM = -5 * coefficientPWM;
  rightPWM = -5 * coefficientPWM;
  distance = -3;
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

  // ジャイロセンサをリセットする
  ResetGyroSensor *resetGyroSensor = new ResetGyroSensor();
  commandExecutor->addCommand(resetGyroSensor, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetGyroSensor));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // MeasAngleをリセットする
  ResetMeasAngle *resetMeasAngle = new ResetMeasAngle();
  commandExecutor->addCommand(resetMeasAngle, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetMeasAngle));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  distance = 27;
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
  leftPWM = -7 * coefficientPWM;
  rightPWM = -7 * coefficientPWM;
  distance = -5.2;
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
  commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(5), "releaseWheel");

  // スラローム位置補正ここまで

  // 指示待ち走行ここから

  // 向き調節
  pwm = 8 * coefficientPWMForFacingAngle;
  FacingAngleAbs *facingAngle1 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset);
  commandExecutor->addCommand(facingAngle1, new FinishedCommandPredicate(facingAngle1), GET_VARIABLE_NAME(facingAngle1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 位置調節
  pwm = 12 * coefficientPWM;
  distance = 7;
  Hedgehog *headgehogA = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehogA, new FinishedCommandPredicate(headgehogA), GET_VARIABLE_NAME(headgehogA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 12 * coefficientPWMForCurve;
  radius = 16;
  theta = 40;
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
  radius = 14;
  theta = -40;
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
  distance = 12;
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
  distance = 14;
  Walker *walkerD = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerDPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerD, walkerDPredicate, GET_VARIABLE_NAME(walkerD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -35;
  FacingAngleAbs *facingAngle4 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle4, new FinishedCommandPredicate(facingAngle4), GET_VARIABLE_NAME(facingAngle4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 7 * coefficientPWMForCurve;
  radius = 10;
  theta = 24;
  CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // バック
  leftPWM = -7 * coefficientPWM;
  rightPWM = -7 * coefficientPWM;
  distance = -2;
  Walker *walkerZ = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerZPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerZ, walkerZPredicate, GET_VARIABLE_NAME(walkerZ));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 14;
  theta = 55;
  CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 3;
  Walker *walker5 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 20;
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
  pwm = 12 * coefficientPWM;
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
  radius = 12;
  theta = -40;
  CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 120度左を向く
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -140;
  FacingAngleAbs *facingAngle7 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle7, new FinishedCommandPredicate(facingAngle7), GET_VARIABLE_NAME(facingAngle7));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 10;
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
  distance = 20;
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
  // ↓ここから小路↓
  // 1,少し後退
  float leftPow;
  float rightPow;
  leftPow = -15;
  rightPow = -15;
  Walker *walker1 = new Walker(leftPow, rightPow);
  Predicate *predicate1 = new WheelDistancePredicate(-18, robotAPI);
  commandExecutor->addCommand(walker1, predicate1, GET_VARIABLE_NAME(walker1));
  Stopper *stopper1 = new Stopper();
  Predicate *predicateS1 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper1, predicateS1, GET_VARIABLE_NAME(stoppper1));
  // 2,90ど左回転
  CommandAndPredicate *predicate2 = new RotateRobotUseGyroCommandAndPredicate(-90, 5, robotAPI);
  commandExecutor->addCommand(predicate2->getCommand(), predicate2->getPredicate(), GET_VARIABLE_NAME(predicate2->getCommand()));
  Stopper *stopper2 = new Stopper();
  Predicate *predicateS2 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper2, predicateS2, GET_VARIABLE_NAME(stoppper2));
  // 3「黒検知するまで」直進
  leftPow = 20;
  rightPow = 20;
  Walker *walker3 = new Walker(leftPow, rightPow);
  Predicate *predicate3 = new WheelDistancePredicate(50, robotAPI);
  commandExecutor->addCommand(walker3, predicate3, GET_VARIABLE_NAME(walker3));
  Predicate *predicate3b = new ColorPredicate(COLOR_BLACK);
  commandExecutor->addCommand(walker3, predicate3b, GET_VARIABLE_NAME(walker3));
  // 4,運搬物中心に向く
  leftPow = 5;
  rightPow = -5;
  CommandAndPredicate *predicate4 = new RotateRobotUseGyroCommandAndPredicate(100, 10, robotAPI);
  commandExecutor->addCommand(predicate4->getCommand(), predicate4->getPredicate(), GET_VARIABLE_NAME(predicate4->getCommand()));
  Stopper *stopper4 = new Stopper();
  Predicate *predicateS4 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper4, predicateS4, GET_VARIABLE_NAME(stoppper4));
  // 8,灰色で止まり、運搬物中心に向かって直進
  Command *walker = new Walker(10, 10);
  int *r = new int(25);
  int *g = new int(30);
  int *b = new int(40);
  // Predicate *predicate8 = new WheelDistancePredicate(1, robotAPI);
  //   int *r = new int(75);
  // int *g = new int(75);
  // int *b = new int(105);
  Predicate *predicate8 = new RawColorPredicate(r, BETWEEN15, g, BETWEEN15, b, BETWEEN15);
  commandExecutor->addCommand(walker, predicate8, GET_VARIABLE_NAME(walker));
  Stopper *stopper8 = new Stopper();
  Predicate *predicateS8 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
  CommandAndPredicate *predicate8r = new RotateRobotUseGyroCommandAndPredicate(-10, 10, robotAPI);
  commandExecutor->addCommand(predicate8r->getCommand(), predicate8r->getPredicate(), GET_VARIABLE_NAME(predicate8r->getCommand()));
  commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
  leftPow = 20;
  rightPow = 20;
  Walker *walker8 = new Walker(leftPow, rightPow);
  Predicate *predicate8w = new WheelDistancePredicate(5, robotAPI);
  commandExecutor->addCommand(walker8, predicate8w, GET_VARIABLE_NAME(walker8));
  commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
  // 9，左に旋回する。
  leftPow = -5;
  rightPow = 5;
  CommandAndPredicate *predicate9 = new RotateRobotUseGyroCommandAndPredicate(-30, 5, robotAPI);
  commandExecutor->addCommand(predicate9->getCommand(), predicate9->getPredicate(), GET_VARIABLE_NAME(predicate9->getCommand()));
  Stopper *stopper9 = new Stopper();
  Predicate *predicateS9 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper9, predicateS9, GET_VARIABLE_NAME(stoppper9));
  // 10,青丸に向かって直進
  leftPow = 20;
  rightPow = 20;
  Walker *walker10 = new Walker(leftPow, rightPow);
  Predicate *predicate10 = new WheelDistancePredicate(50, robotAPI);
  commandExecutor->addCommand(walker10, predicate10, GET_VARIABLE_NAME(walker10));
  Stopper *stopper10 = new Stopper();
  Predicate *predicateS10 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper10, predicateS10, GET_VARIABLE_NAME(stoppper10));
  // １１ 指定角度右回転（青ラインに向く）
  CommandAndPredicate *predicate11 = new RotateRobotUseGyroCommandAndPredicate(18, 5, robotAPI);
  commandExecutor->addCommand(predicate11->getCommand(), predicate11->getPredicate(), GET_VARIABLE_NAME(predicate11->getCommand()));
  Stopper *stopper11 = new Stopper();
  Predicate *predicateS11 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
  // 12「青検知するまで」直進
  leftPow = 20;
  rightPow = 20;
  Walker *walker12 = new Walker(leftPow, rightPow);
  Predicate *predicate12 = new WheelDistancePredicate(11, robotAPI);
  commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
  Walker *walker12b = new Walker(leftPow, rightPow);
  Predicate *predicate12b = new ColorPredicate(COLOR_BLUE);
  commandExecutor->addCommand(walker12b, predicate12b, GET_VARIABLE_NAME(walker12b));
  commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
  // 13,10ど右回転 ８回
  CommandAndPredicate *predicate13 = new RotateRobotUseGyroCommandAndPredicate(10, 5, robotAPI);
  Walker *walker13S = new Walker(leftPow, rightPow);
  Predicate *predicate13S = new WheelDistancePredicate(1, robotAPI);
  for (int i = 0; i < 8; i++)
  {
    commandExecutor->addCommand(predicate13->getCommand(), predicate13->getPredicate(), GET_VARIABLE_NAME(predicate13->getCommand()));
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    commandExecutor->addCommand(walker13S, predicate13S, GET_VARIABLE_NAME(walker13S));
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
  }
  // leftPow = 20;
  // CommandAndPredicate *predicate13l = new RotateRobotUseGyroCommandAndPredicate(5,5,robotAPI);
  // commandExecutor->addCommand(predicate13l->getCommand(), predicate13l->getPredicate(), GET_VARIABLE_NAME(predicate13l->getCommand()));
  // commandExecutor->addCommand(walker13S, predicate13S, GET_VARIABLE_NAME(walker13S));
  // 14,ガレージに直進（取得した色ごとに分岐させる）colorReadergetColor()で色を取得できる
  Command *dealingWithGarage14 = new DealingWithGarage(garageCardColorPtr, commandExecutor, false);
  Predicate *predicate14 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(dealingWithGarage14, predicate14, GET_VARIABLE_NAME(dealingWithGarage14));

  // ↑ここまで小路↑
}
#endif

#ifdef PIDStraightWalkerTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;
  float r;

  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngleAtSlalom = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngleAtSlalom, new FinishedCommandPredicate(resetArmAngleAtSlalom), GET_VARIABLE_NAME(resetArmAngleAtSlalom));

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  // pwm10
  pwm = 10;
  kp = 0.6265;
  ki = 0.0812;
  kd = 1.9221;
  dt = 1;

  PIDStraightWalker *pidStraightWalker = new PIDStraightWalker(pwm, kp, ki, kd, dt);
  commandExecutor->addCommand(pidStraightWalker, new Predicate(), GET_VARIABLE_NAME(pidStraightWalker));
}
#endif

void runner_task(intptr_t exinf)
{
  ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示
#ifdef StopWhenThrowException
  try
  {
#endif
    commandExecutor->run(); // 走らせる
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif
  ext_tsk();
}

enum ReturnToStartPointState
{
  RTSP_TURNNING_UP,
  RTSP_WALKING_UP,
  RTSP_TURNNING_RIGHT,
  RTSP_WALKING_RIGHT,
  RTSP_FINISH,
};

ReturnToStartPointState returnToStartPointState = RTSP_TURNNING_UP;
Walker *returnToStartPointStraightWalker = new Walker(20, 20);
FacingAngleAbs *facing180 = new FacingAngleAbs(FA_WheelCount, 10, 180);
FacingAngleAbs *facing90 = new FacingAngleAbs(FA_WheelCount, 10, 270);
Predicate *facing180Predicate = new FinishedCommandPredicate(facing180);
Predicate *facing90Predicate = new FinishedCommandPredicate(facing90);
bool initedFacing180 = false;
bool initedFacing90 = false;
colorid_t returnToStartPointEdgeLineColor = COLOR_RED;
bool startedBackToTheFuture = false;

void return_to_start_point_task(intptr_t exinf)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    switch (returnToStartPointState)
    {
    case RTSP_TURNNING_UP:
    {
      if (!initedFacing180)
      {
        StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
        startDededon->run(robotAPI);
        delete startDededon;

        facing180->preparation(robotAPI);
        facing180Predicate->preparation(robotAPI);
        initedFacing180 = true;
        ext_tsk();
        return;
      }
      if (!facing180Predicate->test(robotAPI))
      {
        facing180->run(robotAPI);
        ext_tsk();
        return;
      }
      else
      {
        returnToStartPointState = RTSP_WALKING_UP;
        ext_tsk();
        return;
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
        ext_tsk();
        return;
      }
    }
    break;

    case RTSP_TURNNING_RIGHT:
    {
      if (!initedFacing90)
      {
        facing90->preparation(robotAPI);
        facing90Predicate->preparation(robotAPI);
        initedFacing90 = true;
        ext_tsk();
        return;
      }
      if (!facing90Predicate->test(robotAPI))
      {
        facing90->run(robotAPI);
        ext_tsk();
        return;
      }
      else
      {
        returnToStartPointState = RTSP_WALKING_RIGHT;
        ext_tsk();
        return;
      }
    }
    break;

    case RTSP_WALKING_RIGHT:
    {
      returnToStartPointStraightWalker->run(robotAPI);
      ext_tsk();
      return;
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

#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif
  ext_tsk();
}

enum BTCommand
{
  BTC_EMERGENCY_STOP = 's',
  BTC_RETURN_TO_START_POINT = 'r',
  BTC_NEXT_COMMAND = 'n',
};

void emergencyStop()
{
  stp_cyc_all();
  commandExecutor->emergencyStop();
}

void listen_bluetooth_command_task(intptr_t exinf)
{
#ifdef EnableBluetooth

#ifdef StopWhenThrowException
  try
  {
#endif

    unsigned char bluetoothCommand = fgetc(bt);
    switch (bluetoothCommand)
    {
    case BTC_EMERGENCY_STOP:
    {
      emergencyStop();
      Stopper *stopper = new Stopper();
      stopper->run(robotAPI);
      delete stopper;
      break;
    }
    case BTC_RETURN_TO_START_POINT:
    {
      if (!startedBackToTheFuture)
      {
        startedBackToTheFuture = true;

        const uint32_t sleepDuration = 100 * 1000;

        emergencyStop();

        vector<string> messageLines;
        messageLines.push_back("STARTED Back to the future");
        PrintMessage *printMessage = new PrintMessage(messageLines, true);
        printMessage->run(robotAPI);
        robotAPI->getClock()->sleep(sleepDuration);
        delete printMessage;

        StartCyc *startBackToTheFuture = new StartCyc(RETURN_TO_START_POINT_CYC);
        startBackToTheFuture->run(robotAPI);
        delete startBackToTheFuture;
        break;
      }
    }
    case BTC_NEXT_COMMAND:
    {
      commandExecutor->nextCommand();
      break;
    }
    default:
      break;
    }
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif
#endif

  ext_tsk();
};

CommandExecutor *singASongCommandExecutor;
void sing_a_song_task(intptr_t exinf)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    singASongCommandExecutor->run();
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif

  ext_tsk();
};

vector<Note *> dededon = generateDededon();
CommandExecutor *dededonCommandExecutor;
void dededon_task(intptr_t exinf)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    dededonCommandExecutor->run();
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif

  ext_tsk();
};

void initDededon()
{
  for (int i = 0; i < ((int)dededon.size()); i++)
  {
    dededonCommandExecutor->addCommand(dededon[i], new FinishedCommandPredicate(dededon[i]), "");
  }
};

void initSong(int loop)
{
  for (int j = 0; j < loop; j++)
  {
    for (int i = 0; i < ((int)song.size()); i++)
    {
      singASongCommandExecutor->addCommand(song[i], new FinishedCommandPredicate(song[i]), "");
    }
  }
};

#if defined(TrueCourceOkiharaModeRegional) | defined(TrueCourceKomichiModeRegional)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngle = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngle, new FinishedCommandPredicate(resetArmAngle), GET_VARIABLE_NAME(resetArmAngle));

#ifdef TrueCourceKomichiModeRegional
  // ↓ここから小路↓
  float Kpwm;
  float Kkp;
  float Kki;
  float Kkd;
  float Kdt;

  int KleftPow;
  int KrightPow;
  int Kdistance;

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

  float leftPow;
  float rightPow;

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

//#define SonarStarter
#ifdef SonarStarter
  Walker *sonarStandby = new Walker(leftPow, rightPow);
  Predicate *sonarStater = new SonarDistancePredicate(10, true);
  commandExecutor->addCommand(sonarStandby, sonarStater, GET_VARIABLE_NAME(sonarStandby));
#endif

  Kpwm = 22;
  Kkp = 0.7;
  Kki = 0.2;
  Kkd = 0.7;
  Kdt = 1;
  Kdistance = 8;
  PIDTracer *KbananaPIDTracer = new PIDTracer(RIGHT_TRACE, Kpwm, Kkp, Kki, Kkd, Kdt);
  calibrator->addPIDTracer(KbananaPIDTracer);

  KleftPow = 0;
  KrightPow = 0;
  Walker *KwalkerS = new Walker(KleftPow, KrightPow);
  // Predicate *predicate0 = new NumberOfTimesPredicate(4);
  Predicate *Kpredicate0 = new WheelDistancePredicate(Kdistance, robotAPI);
  commandExecutor->addCommand(KbananaPIDTracer, Kpredicate0, GET_VARIABLE_NAME(KbananaPIDTracer));
  //第一直進
  KleftPow = 50;
  KrightPow = 50;
  KwalkerS = new Walker(KleftPow, KrightPow);
  // Predicate *predicate1 = new WheelDistancePredicate(40, robotAPI);
  Predicate *Kpredicate1 = new WheelDistancePredicate(20, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate1, GET_VARIABLE_NAME(KwalkerS));
  //第二カーブ
  KleftPow = 50;
  KrightPow = 10;
  Walker *Kwalker2 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate2 = new WheelDistancePredicate(24, robotAPI);
  commandExecutor->addCommand(Kwalker2, Kpredicate2, GET_VARIABLE_NAME(Kwalker2));
  //第三直進
  KleftPow = 50;
  KrightPow = 50;
  Walker *Kwalker3 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate3 = new WheelDistancePredicate(52, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate3, GET_VARIABLE_NAME(KwalkerS));
  //第四カーブ
  KleftPow = 50;
  KrightPow = 10;
  Walker *Kwalker4 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate4 = new WheelDistancePredicate(25, robotAPI);
  commandExecutor->addCommand(Kwalker4, Kpredicate4, GET_VARIABLE_NAME(Kwalker4));
  // 5直進
  Predicate *Kpredicate5 = new WheelDistancePredicate(10, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate5, GET_VARIABLE_NAME(KwalkerS));
  //第6カーブ,mid1
  KleftPow = 30;
  KrightPow = 50;
  Walker *Kwalker6 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate6 = new WheelDistancePredicate(70, robotAPI);
  commandExecutor->addCommand(Kwalker6, Kpredicate6, GET_VARIABLE_NAME(Kwalker6));
  Predicate *KpredicateMid1 = new WheelDistancePredicate(9, robotAPI);
  commandExecutor->addCommand(KwalkerS, KpredicateMid1, GET_VARIABLE_NAME(KwalkerS));
  // OK第7　一度目交差点から丸一周
  KleftPow = 50;
  KrightPow = 36;
  Walker *Kwalker7 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate7 = new WheelDistancePredicate(340, robotAPI);
  commandExecutor->addCommand(Kwalker7, Kpredicate7, GET_VARIABLE_NAME(Kwalker7));
  //第8　2度目交差点から抜ける
  KleftPow = 20;
  KrightPow = 50;
  Walker *Kwalker8 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate8 = new WheelDistancePredicate(8, robotAPI);
  commandExecutor->addCommand(Kwalker8, Kpredicate8, GET_VARIABLE_NAME(Kwalker8));
  // 9直進
  Predicate *Kpredicate9 = new WheelDistancePredicate(42, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate9, GET_VARIABLE_NAME(KwalkerS));
  // 10カーブ
  KleftPow = 8;
  KrightPow = 50;
  Walker *Kwalker10 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate10 = new WheelDistancePredicate(5, robotAPI);
  commandExecutor->addCommand(Kwalker8, Kpredicate10, GET_VARIABLE_NAME(Kwalker8));
  // 11直進
  Predicate *Kpredicate11 = new WheelDistancePredicate(65, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate11, GET_VARIABLE_NAME(KwalkerS));
  // 12Uカーブ
  KleftPow = 50;
  KrightPow = 12;
  Walker *Kwalker12 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate12 = new WheelDistancePredicate(26, robotAPI);
  commandExecutor->addCommand(Kwalker12, Kpredicate12, GET_VARIABLE_NAME(Kwalker12));
  Kpredicate12 = new WheelDistancePredicate(25, robotAPI);
  Predicate *KpredicateS12 = new WheelDistancePredicate(27, robotAPI);
  commandExecutor->addCommand(KwalkerS, KpredicateS12, GET_VARIABLE_NAME(KwalkerS));
  commandExecutor->addCommand(Kwalker12, Kpredicate12, GET_VARIABLE_NAME(Kwalker12));
  commandExecutor->addCommand(KwalkerS, Kpredicate11, GET_VARIABLE_NAME(KwalkerS));
  // IF１３コースに沿って直進
  Predicate *Kpredicate13S = new WheelDistancePredicate(155, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate13S, GET_VARIABLE_NAME(KwalkerS));
  KleftPow = 13;
  KrightPow = 50;
  Walker *Kwalker13 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate13 = new WheelDistancePredicate(7, robotAPI);
  commandExecutor->addCommand(Kwalker13, Kpredicate13, GET_VARIABLE_NAME(Kwalker13));
  Predicate *Kpredicate13S2 = new WheelDistancePredicate(20, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate13S2, GET_VARIABLE_NAME(KwalkerS));
  //   //13ゴールに向かって直進
  //  leftPow = 31;
  // rightPow = 100;
  // Walker *walker13 = new Walker(leftPow, rightPow);
  // Predicate *predicate13 = new WheelDistancePredicate(2, robotAPI);
  // commandExecutor->addCommand(walker13, predicate13, GET_VARIABLE_NAME(walker13));
  // Predicate *predicate13S = new WheelDistancePredicate(151, robotAPI);
  // commandExecutor->addCommand(walkerS, predicate13S, GET_VARIABLE_NAME(walkerS));
  // Predicate *predicate13t = new WheelDistancePredicate(36, robotAPI);
  // commandExecutor->addCommand(walker13, predicate13t, GET_VARIABLE_NAME(walker13));
  // 14黒キャッチしてから　ライントレース
  /*
  leftPow = 20;
  rightPow = 20;
  Walker *walker14S = new Walker(leftPow, rightPow);
  Predicate *predicate14c = new ColorPredicate(COLOR_BLACK);
  commandExecutor->addCommand(walker14S, predicate14c, GET_VARIABLE_NAME(walker14S));
  */

  Kpwm = 10;
  float Kangle = -45;
  FacingAngleAbs *KfacingAngle45 = new FacingAngleAbs(FA_Gyro, Kpwm, Kangle);
  commandExecutor->addCommand(KfacingAngle45, new FinishedCommandPredicate(KfacingAngle45), GET_VARIABLE_NAME(KfacingAngle45));

  KleftPow = 20;
  KrightPow = 20;
  Walker *KlowWalker = new Walker(KleftPow, KrightPow);
  Predicate *KblackPredicate = new BlackPredicate();
  commandExecutor->addCommand(KlowWalker, KblackPredicate, GET_VARIABLE_NAME(KlowWalker));

  Kpwm = 15;
  Kkp = 0.2;
  Kki = 0.1;
  Kkd = 0.2;
  Kdt = 1;
  ColorPIDTracer *KcolorPIDTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, Kpwm, Kkp, Kki, Kkd, Kdt);
  calibrator->addColorPIDTracer(KcolorPIDTracer);
  commandExecutor->addCommand(KcolorPIDTracer, new BlueEdgePredicate(), GET_VARIABLE_NAME(KcolorPIDTracer));

  // ↑ここまで小路↑

#endif

// RGBRawReaderModeの場合のcommandExecutor初期化処理
#ifdef RGBRawReaderMode
  void initializeCommandExecutor(CommandExecutor * commandExecutor, RobotAPI * robotAPI)
  {
    // rgbRawReaderの初期化とCommandExecutorへの追加
    RGBRawReader *rgbRawReader = new RGBRawReader();
    Predicate *startButtonPredicate = new StartButtonPredicate();
    commandExecutor->addCommand(rgbRawReader, startButtonPredicate, GET_VARIABLE_NAME(rgbRawReader));
  }
#endif
// ColorIDReaderModeの場合のcommandExecutor初期化処理
#ifdef ColorIDReaderMode
  void initializeCommandExecutor(CommandExecutor * commandExecutor, RobotAPI * robotAPI)
  {
    // rgbRawReaderの初期化とCommandExecutorへの追加
    ColorIDReader *reader = new ColorIDReader();
    Predicate *startButtonPredicate = new StartButtonPredicate();
    commandExecutor->addCommand(reader, startButtonPredicate, GET_VARIABLE_NAME(reader));
  }
#endif
#ifdef BrightnessReaderMode
  void initializeCommandExecutor(CommandExecutor * commandExecutor, RobotAPI * robotAPI)
  {
    BrightnessReader *reader = new BrightnessReader();
    commandExecutor->addCommand(reader, new Predicate(), GET_VARIABLE_NAME(reader));
  }
#endif
// Rotate360TestModeの場合のcommandExecutor初期化処理
#if defined(Rotate360TestMode)
  void initializeCommandExecutor(CommandExecutor * commandExecutor, RobotAPI * robotAPI)
  {
    int motorRotateAngle = 540; // ここの値をいじってはかって
    // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
    Predicate *startButtonPredicate = new StartButtonPredicate();
    commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
    // 走行体回転コマンドの初期化とCommandExecutorへの追加
    float pwm = 10;
    Walker *walker = new Walker(pwm, -pwm); // 右に向く
    Predicate *walkerPredicate = new MotorCountPredicate(robotAPI->getLeftWheel(), motorRotateAngle);
    commandExecutor->addCommand(walker, walkerPredicate, GET_VARIABLE_NAME(walker));
    // 停止コマンドの初期化とCommandExecutorへの追加
    Stopper *stopper = new Stopper();
    Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  }
#endif
// RotateTestModeの場合のcommandExecutor初期化処理
#ifdef RotateTestMode
  void initializeCommandExecutor(CommandExecutor * commandExecutor, RobotAPI * robotAPI)
  {
    // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
    Predicate *startButtonPredicate = new StartButtonPredicate();
    commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
    // 走行体回転コマンドの初期化とCommandExecutorへの追加
    float angle = 10;
    float pwm = 15;
    RotateRobotCommandAndPredicate *commandAndPredicate = new RotateRobotCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), "rotateRobot");
    // 停止コマンドの初期化とCommandExecutorへの追加
    Stopper *stopper = new Stopper();
    Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
  }
#endif

#ifdef TrueCourceOkiharaModeRegional
  // ↓ここから沖原↓

  // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  float bananaDistance = 158;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  float orangeDistance = 68;      // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  float starFruitsDistance = 5;   // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  float cherryDistance = 14;      // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  float waterMelonDistance = 314; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  float bokChoyDistance = 30;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  float dorianDistance = 14;      // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  float asparagusDistance = 41;
  float radishDistance = 41;
  float melonDistance = 118;     // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  float cucumberDistance = 149;  // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  float strawberryDistance = 55; // ゴールまで。いちご好き。ライントレースする。
  float cabbageDistance = 127;

  /*
  int sceneBananaMotorCountPredicateArg = 1750;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int sceneOrangeMotorCountPredicateArg = 2500;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2550; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2700;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
#ifdef TrueRightCourceMode
  int sceneBokChoyMotorCountPredicateArg = 6700;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 6860;      // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneAsparagusMotorCountPredicateArg = 7310;   // ドリアン終了後の１つ目の直線
  int sceneRadishMotorCountPredicateArg = 7760;      // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
  int sceneMelonMotorCountPredicateArg = 9030;       // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneCucumberMotorCountPredicateArg = 10705;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 11310; // ゴールまで。いちご好き。ライントレースする。
  int sceneCabbageMotorCountpredicateArg = 12710;    // ゴールまで。
#else
  int sceneBokChoyMotorCountPredicateArg = 6490;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 6650;      // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneAsparagusMotorCountPredicateArg = 7100;   // ドリアン終了後の１つ目の直線
  int sceneRadishMotorCountPredicateArg = 7550;      // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
  int sceneMelonMotorCountPredicateArg = 8850;       // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneCucumberMotorCountPredicateArg = 10495;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 11100; // ゴールまで。いちご好き。ライントレースする。
  int sceneCabbageMotorCountpredicateArg = 12500;    // ゴールまで。
#endif

  float distanceTemp = 0;
  int bananaDistance = (sceneBananaMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += bananaDistance;
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
  int cucumberDistance = (sceneCucumberMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cucumberDistance;
  int strawberryDistance = (sceneStrawberryMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += strawberryDistance;
  int cabbageDistance = (sceneCabbageMotorCountpredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
  distanceTemp += cabbageDistance;

  printf("以下出力された値をbananaDistanceとかに入れていって。");
  printf("%sDistance: %10.f\n", "Banana", float(bananaDistance));
  printf("%sDistance: %10.f\n", "Orange", float(orangeDistance));
  printf("%sDistance: %10.f\n", "StarFruits", float(starFruitsDistance));
  printf("%sDistance: %10.f\n", "Cherry", float(cherryDistance));
  printf("%sDistance: %10.f\n", "WaterMelon", float(waterMelonDistance));
  printf("%sDistance: %10.f\n", "BokChoy", float(bokChoyDistance));
  printf("%sDistance: %10.f\n", "Dorian", float(dorianDistance));
  printf("%sDistance: %10.f\n", "Asparagus", float(asparagusDistance));
  printf("%sDistance: %10.f\n", "Radish", float(radishDistance));
  printf("%sDistance: %10.f\n", "Melon", float(melonDistance));
  printf("%sDistance: %10.f\n", "Cucumber", float(cucumberDistance));
  printf("%sDistance: %10.f\n", "Strawberry", float(strawberryDistance));
  printf("%sDistance: %10.f\n", "CabbageDistance", float(cabbageDistance));
  printf("以上");
  */

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

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

  // BananaPIDTracerの初期化とCommandExecutorへの追加
  pwm = 22;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
  calibrator->addPIDTracer(bananaPIDTracer);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 25;
  kp = 0.5;
  ki = 0.2;
  kd = 0.45;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
  calibrator->addPIDTracer(orangePIDTracer);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

  // StarFruitsWalkerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 16;
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
  pwm = 25;
  kp = 0.5;
  ki = 0.2;
  kd = 0.5;
  dt = 1;
  angle = 335;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  // Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
  Predicate *predicateWaterMelon = new ORPreicate(new WheelDistancePredicate(waterMelonDistance, robotAPI), new FacingRobotUseWheelPredicate(angle));
  calibrator->addPIDTracer(waterMelonPIDTracer);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

  // BokChoyWalkerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 20;
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

  // AsparagusPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *asparagusPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateAsparagus = new WheelDistancePredicate(asparagusDistance, robotAPI);
  calibrator->addPIDTracer(asparagusPIDTracer);
  commandExecutor->addCommand(asparagusPIDTracer, predicateAsparagus, GET_VARIABLE_NAME(asparagusPIDTracer));

  // RadishPIDTracerの初期化とCommandExecutorへの追加
  pwm = 25;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *radishPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateRadish = new WheelDistancePredicate(radishDistance, robotAPI);
  calibrator->addPIDTracer(radishPIDTracer);
  commandExecutor->addCommand(radishPIDTracer, predicateRadish, GET_VARIABLE_NAME(radishPIDTracer));

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

  pwm = 30;
  kp = 0.5;
  ki = 0.2;
  kd = 0.5;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
  calibrator->addPIDTracer(cucumberPIDTracer);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
  calibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

  // CabbagePIDTracerの初期化とCommandExecutorへの追加
  pwm = 30;
  kp = 0.5;
  ki = 0.2;
  kd = 0.5;
  dt = 1;
  PIDTracer *cabbagePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
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
#endif
  // ↑ここまで沖原↑

  // ↓ここから実方↓
  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "Stopper");

  // ガレージカードの色取得用ColorReader
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

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
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
#else
  kp = 0.19;
  ki = 0.15;
  kd = 0.19;
  dt = 1;
#endif
  pwm = 15 * coefficientPWM;
  ColorPIDTracer *pidTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 10 * coefficientPWM;
  ColorPIDTracer *lowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  pwm = 5 * coefficientPWM;
  ColorPIDTracer *verryLowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
  calibrator->addColorPIDTracer(pidTracer);
  calibrator->addColorPIDTracer(lowPWMTracer);
  calibrator->addColorPIDTracer(verryLowPWMTracer);
#ifdef SimulatorMode
  float targetBrightness = 20;
  rgb_raw_t targetRGB;
  targetRGB.r = blackWhiteEdgeR;
  targetRGB.g = 60;
  targetRGB.b = 60;
  pidTracer->setTargetColor(targetRGB);
  lowPWMTracer->setTargetColor(targetRGB);
#endif
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
  // カラーセンサの乗ったアームの角度を調節する
  ResetArmAngle *resetArmAngleAtSlalom = new ResetArmAngle();
  commandExecutor->addCommand(resetArmAngleAtSlalom, new FinishedCommandPredicate(resetArmAngleAtSlalom), GET_VARIABLE_NAME(resetArmAngleAtSlalom));

  // 1.5秒止める。BrightnessからColorへの切り替えのために。
  commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));
  // uint64_t waitDurationUsec = 1000 * 1000;
  // commandExecutor->addCommand(stopper, new TimerPredicate(waitDurationUsec), "wait switch mode brightness to row color");

  // PIDトレースで青線まで進む
  Predicate *distancePredicate = new WheelDistancePredicate(40, robotAPI);
  commandExecutor->addCommand(pidTracer, distancePredicate, GET_VARIABLE_NAME(lowPWMTracer));

  // PIDトレースで青線まで進む
  Predicate *pidTracerPredicate = new BlueEdgePredicate();
  commandExecutor->addCommand(pidTracer, pidTracerPredicate, GET_VARIABLE_NAME(lowPWMTracer));

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
  numberOfTime = 80;
  leftPWM = 7 * coefficientPWM;
  rightPWM = 7 * coefficientPWM;
  Command *lowWalker = new Walker(leftPWM, rightPWM);
  commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
  commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

  // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
  leftPWM = -5 * coefficientPWM;
  rightPWM = -5 * coefficientPWM;
  distance = -3;
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

  // ジャイロセンサをリセットする
  ResetGyroSensor *resetGyroSensor = new ResetGyroSensor();
  commandExecutor->addCommand(resetGyroSensor, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetGyroSensor));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // MeasAngleをリセットする
  ResetMeasAngle *resetMeasAngle = new ResetMeasAngle();
  commandExecutor->addCommand(resetMeasAngle, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetMeasAngle));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
  distance = 27;
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
  leftPWM = -7 * coefficientPWM;
  rightPWM = -7 * coefficientPWM;
  // distance = -5.2;
  distance = -3.3; // 大会で変更
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

  // スラローム位置補正ここまで

  // 指示待ち走行ここから

  // 向き調節
  pwm = 8 * coefficientPWMForFacingAngle;
  FacingAngleAbs *facingAngle1 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset);
  commandExecutor->addCommand(facingAngle1, new FinishedCommandPredicate(facingAngle1), GET_VARIABLE_NAME(facingAngle1));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 位置調節
  pwm = 12 * coefficientPWM;
  distance = 7;
  Hedgehog *headgehogA = new Hedgehog(distance, pwm);
  commandExecutor->addCommand(headgehogA, new FinishedCommandPredicate(headgehogA), GET_VARIABLE_NAME(headgehogA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 12 * coefficientPWMForCurve;
  radius = 16;
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
  radius = 18;
  theta = -30;
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
  distance = 8;
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
  distance = 11;
  Walker *walkerD = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerDPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerD, walkerDPredicate, GET_VARIABLE_NAME(walkerD));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 向き調節
  pwm = 7 * coefficientPWMForFacingAngle;
  angle = -35;
  FacingAngleAbs *facingAngle4 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
  commandExecutor->addCommand(facingAngle4, new FinishedCommandPredicate(facingAngle4), GET_VARIABLE_NAME(facingAngle4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 7 * coefficientPWMForCurve;
  radius = 10;
  theta = 24;
  CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // バック
  leftPWM = -7 * coefficientPWM;
  rightPWM = -7 * coefficientPWM;
  distance = -2;
  Walker *walkerZ = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walkerZPredicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walkerZ, walkerZPredicate, GET_VARIABLE_NAME(walkerZ));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 14;
  theta = 55;
  CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
  commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // 直進
  leftPWM = 10 * coefficientPWM;
  rightPWM = 10 * coefficientPWM;
  distance = 3;
  Walker *walker5 = new Walker(leftPWM, rightPWM);
  WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
  commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

  // カーブ
  pwm = 8 * coefficientPWMForCurve;
  radius = 20;
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
  int diff = 7;
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
  pwm = 12 * coefficientPWM;
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
  // ↓ここから小路↓
  // 1,少し後退
  leftPow = -15;
  rightPow = -15;
  Walker *walker1 = new Walker(leftPow, rightPow);
  Predicate *predicate1 = new WheelDistancePredicate(-18, robotAPI);
  commandExecutor->addCommand(walker1, predicate1, GET_VARIABLE_NAME(walker1));
  Stopper *stopper1 = new Stopper();
  Predicate *predicateS1 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper1, predicateS1, GET_VARIABLE_NAME(stoppper1));
  // 2,90ど左回転
  CommandAndPredicate *predicate2 = new RotateRobotUseGyroCommandAndPredicate(-90, 5, robotAPI);
  commandExecutor->addCommand(predicate2->getCommand(), predicate2->getPredicate(), GET_VARIABLE_NAME(predicate2->getCommand()));
  Stopper *stopper2 = new Stopper();
  Predicate *predicateS2 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper2, predicateS2, GET_VARIABLE_NAME(stoppper2));
  // 3「黒検知するまで」直進
  leftPow = 20;
  rightPow = 20;
  Walker *walker3 = new Walker(leftPow, rightPow);
  Predicate *predicate3 = new WheelDistancePredicate(50, robotAPI);
  commandExecutor->addCommand(walker3, predicate3, GET_VARIABLE_NAME(walker3));
  Predicate *predicate3b = new ColorPredicate(COLOR_BLACK);
  commandExecutor->addCommand(walker3, predicate3b, GET_VARIABLE_NAME(walker3));
  // 4,運搬物中心に向く
  leftPow = 5;
  rightPow = -5;
  CommandAndPredicate *predicate4 = new RotateRobotUseGyroCommandAndPredicate(100, 10, robotAPI);
  commandExecutor->addCommand(predicate4->getCommand(), predicate4->getPredicate(), GET_VARIABLE_NAME(predicate4->getCommand()));
  Stopper *stopper4 = new Stopper();
  Predicate *predicateS4 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper4, predicateS4, GET_VARIABLE_NAME(stoppper4));
  // 8,灰色で止まり、運搬物中心に向かって直進
  Command *walker = new Walker(10, 10);
  int *r = new int(25);
  int *g = new int(30);
  int *b = new int(40);
  // Predicate *predicate8 = new WheelDistancePredicate(1, robotAPI);
  //   int *r = new int(75);
  // int *g = new int(75);
  // int *b = new int(105);
  Predicate *predicate8 = new RawColorPredicate(r, BETWEEN15, g, BETWEEN15, b, BETWEEN15);
  commandExecutor->addCommand(walker, predicate8, GET_VARIABLE_NAME(walker));
  Stopper *stopper8 = new Stopper();
  Predicate *predicateS8 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
  CommandAndPredicate *predicate8r = new RotateRobotUseGyroCommandAndPredicate(-10, 10, robotAPI);
  commandExecutor->addCommand(predicate8r->getCommand(), predicate8r->getPredicate(), GET_VARIABLE_NAME(predicate8r->getCommand()));
  commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
  leftPow = 20;
  rightPow = 20;
  Walker *walker8 = new Walker(leftPow, rightPow);
  Predicate *predicate8w = new WheelDistancePredicate(5, robotAPI);
  commandExecutor->addCommand(walker8, predicate8w, GET_VARIABLE_NAME(walker8));
  commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
  // 9，左に旋回する。
  leftPow = -5;
  rightPow = 5;
  CommandAndPredicate *predicate9 = new RotateRobotUseGyroCommandAndPredicate(-30, 5, robotAPI);
  commandExecutor->addCommand(predicate9->getCommand(), predicate9->getPredicate(), GET_VARIABLE_NAME(predicate9->getCommand()));
  Stopper *stopper9 = new Stopper();
  Predicate *predicateS9 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper9, predicateS9, GET_VARIABLE_NAME(stoppper9));
  // 10,青丸に向かって直進
  leftPow = 20;
  rightPow = 20;
  Walker *walker10 = new Walker(leftPow, rightPow);
  Predicate *predicate10 = new WheelDistancePredicate(50, robotAPI);
  commandExecutor->addCommand(walker10, predicate10, GET_VARIABLE_NAME(walker10));
  Stopper *stopper10 = new Stopper();
  Predicate *predicateS10 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper10, predicateS10, GET_VARIABLE_NAME(stoppper10));
  // １１ 指定角度右回転（青ラインに向く）
  CommandAndPredicate *predicate11 = new RotateRobotUseGyroCommandAndPredicate(18, 5, robotAPI);
  commandExecutor->addCommand(predicate11->getCommand(), predicate11->getPredicate(), GET_VARIABLE_NAME(predicate11->getCommand()));
  Stopper *stopper11 = new Stopper();
  Predicate *predicateS11 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
  // 12「青検知するまで」直進
  leftPow = 20;
  rightPow = 20;
  Walker *walker12 = new Walker(leftPow, rightPow);
  Predicate *predicate12 = new WheelDistancePredicate(11, robotAPI);
  commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
  Walker *walker12b = new Walker(leftPow, rightPow);
  Predicate *predicate12b = new ColorPredicate(COLOR_BLUE);
  commandExecutor->addCommand(walker12b, predicate12b, GET_VARIABLE_NAME(walker12b));
  commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
  // 13,10ど右回転 ８回
  CommandAndPredicate *predicate13 = new RotateRobotUseGyroCommandAndPredicate(10, 5, robotAPI);
  Walker *walker13S = new Walker(leftPow, rightPow);
  Predicate *predicate13S = new WheelDistancePredicate(1, robotAPI);
  for (int i = 0; i < 8; i++)
  {
    commandExecutor->addCommand(predicate13->getCommand(), predicate13->getPredicate(), GET_VARIABLE_NAME(predicate13->getCommand()));
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    commandExecutor->addCommand(walker13S, predicate13S, GET_VARIABLE_NAME(walker13S));
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
  }
  // leftPow = 20;
  // CommandAndPredicate *predicate13l = new RotateRobotUseGyroCommandAndPredicate(5,5,robotAPI);
  // commandExecutor->addCommand(predicate13l->getCommand(), predicate13l->getPredicate(), GET_VARIABLE_NAME(predicate13l->getCommand()));
  // commandExecutor->addCommand(walker13S, predicate13S, GET_VARIABLE_NAME(walker13S));
  // 14,ガレージに直進（取得した色ごとに分岐させる）colorReadergetColor()で色を取得できる
  Command *dealingWithGarage14 = new DealingWithGarage(garageCardColorPtr, commandExecutor, false);
  Predicate *predicate14 = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(dealingWithGarage14, predicate14, GET_VARIABLE_NAME(dealingWithGarage14));

  // ↑ここまで小路↑
}
#endif

#if defined(TrueCourceSanekataModeCS) | defined(TrueCourceOkiharaModeCS)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // wheelDiameter = 10.5; // これは実方機体
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
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();

// ↓ここから沖原↓
#ifdef SanekataCanNotGoal
  if (false)
#endif
#ifdef TrueCourceOkiharaModeCS
  // ↓ここから沖原↓
  {
    // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
    // そのシーンが終了する距離の定義。
    // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
    // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
    /*
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
    */

    int sceneCarrotMotorCountPredicateArg = 750;      // スタートから最初のカーブ終わるまで
    int sceneBananaMotorCountPredicateArg = 1100;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
    int scenePeachMotorCountPredicateArg = 1540;      // バナナとオレンジの間の小さいカーブ
    int sceneOrangeMotorCountPredicateArg = 2450;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
    int sceneStarFruitsMotorCountPredicateArg = 2540; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
    int sceneCherryMotorCountPredicateArg = 2750;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
    int sceneBokChoyMotorCountPredicateArg = 6325;    // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
    int sceneDorianMotorCountPredicateArg = 6650;     // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneAsparagusMotorCountPredicateArg = 7100;  // ドリアン終了後の１つ目の直線
    int sceneRadishMotorCountPredicateArg = 7390;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    int sceneMelonMotorCountPredicateArg = 7840;      // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
    int sceneLemonMotorCountPredicateArg = 8690;
    int sceneCucumberMotorCountPredicateArg = 10505;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
    int sceneStrawberryMotorCountPredicateArg = 11300; // ゴールまで。いちご好き。ライントレースする。
    int sceneCabbageMotorCountpredicateArg = 12000;    // ゴールまで。

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

    /*
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
      */

    float pwm;
    float kp;
    float ki;
    float kd;
    int dt;
    float r;
    float leftPow;
    float rightPow;

    // CarrotPIDTracerの初期化とCommandExecutorへの追加
    //下記コメントアウト箇所アンパイ

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
    //アンパイ

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
    kp = 1.1;
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

    //下記はアンパイ
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
    kp = 1.0;
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

    pwm = 40;
    kp = 1.0;
    ki = 0;
    kd = 4.0;
    dt = 1;
    r = 0;

    /*
    pwm = 30;
    kp = 0.75;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;
    */

    PIDTracerV2 *strawberryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
    calibrator->addPIDTracer(strawberryPIDTracer);
    commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

    // CabbagePIDTracerの初期化とCommandExecutorへの追加

    pwm = 60;
    kp = 1.0;
    ki = 0;
    kd = 2.0;
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
// ↑ここまで沖原↑
#endif
#ifdef TrueCourceSanekataModeCS
  // ↓ここから実方↓
  {
    // int orangePlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    // int orangePlan = 2; // 弱めのPDでRに頼った走行
    int orangePlan = 3;
    int cherryPlan = 1; // 強めのPD、なるべく弱めのRな走行
    // int cherryPlan = 2; // 弱めのPDでRに頼った走行
    // int cherryPlan = 3;// まあまあのPDでRに頼った走行
    // int waterMelonPlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    // int waterMelonPlan = 2; // 弱めのPDでRに頼った走行
    // int waterMelonPlan = 3; // まあまあのPDでRに頼った走行
    int waterMelonPlan = 4; // 弱めのPIDでRに頼った走行
    // int dorianPlan = 1; // まぁまぁなPID走行 iを使うと安定性が下がるのでplan2, plan3を用意しました
    int dorianPlan = 2; // 弱めのPD走行
    // int dorianPlan = 3; // 強めのPD走行
    SetPWMCoefficient *setPWMCoefficient = new SetPWMCoefficient();
    // commandExecutor->addCommand(setPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(setPWMCoefficient));

    bool facingAngleAtStarFruits = false;
    bool facingAngleAtBokChoy = false;

    bool useAnglePredicateAtWaterMelon = true;
    bool useAnglePredicateAtOrange = true;

    float pmanDistance = 34;
    float carrotDistance = 32;
    float bananaDistance = 34;
    float peachDistance = 34;
    float orangeDistance = 72;
    float starFruitsDistance = 12;
    float cherryDistance = 60;
    float waterMelonDistance = 271.5;
    float bokChoyDistance = 15;
    float dorianDistance = 40;
    float hassakuDistance = 35;
    float radishDistance = 34;
    float melonDistance = 36;
    float nutsDistance = 10;
    float lemonDistance = 37;
    float cucumberDistance = 189;
    float strawberryDistance = 45;
    float cabbageDistance = 100;

    /*
    #ifdef Right // 学校のコース伸びた説
    orangeDistance += 1.5;
    waterMelonDistance += 5;
    #endif
    */

    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;
    float r;
    float radius;
    float theta;

    uint64_t waitFaUsec = 500000;

    FacingAngleMode facingAngleMode = FA_WheelCount;
    float angle;
    float faKp = 0.7;
    float faKi = 0;
    float faKd = 0.7;
    float faDt = 1;

    float carrotPWM = 60;
    float carrotKp = 0.35;
    float carrotKi = 0.015; // 0.12;
    float carrotKd = carrotKp * 3;
    float carrotDt = 0.4;
    float carrotR = 55;

    // PmanPIDTracerの初期化とCommandExecutorへの追加
    pwm = 25;
    kp = 0.5;
    ki = 0;
    kd = 1.4;
    dt = 1;
    r = 0;
    PIDTracerV2 *pmanPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatePman = new WheelDistancePredicate(pmanDistance, robotAPI);
    commandExecutor->addCommand(pmanPIDTracer, predicatePman, GET_VARIABLE_NAME(pmanPIDTracer));
    calibrator->addPIDTracer(pmanPIDTracer);

    // CarrotPIDTracerの初期化とCommandExecutorへの追加
    /*
    pwm = 50;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = 30;
    */
    // 距離依存をへらすために速度を落としてAndPredicateを使います
    pwm = 25;
    kp = 0.5;
    ki = 0;
    kd = 1.4;
    dt = 1;
    r = 15;
    angle = 90 - 20;
    PIDTracerV2 *carrotPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    // Predicate *predicateCarrot = new WheelDistancePredicate(carrotDistance, robotAPI);
    Predicate *predicateCarrot = new ANDPredicate(new WheelDistancePredicate(carrotDistance, robotAPI), new FacingRobotUseWheelPredicate(angle));
    commandExecutor->addCommand(carrotPIDTracer, predicateCarrot, GET_VARIABLE_NAME(carrotPIDTracer));
    calibrator->addPIDTracer(carrotPIDTracer);

    // BananaPIDTracerの初期化とCommandExecutorへの追加
    pwm = 45;
    kp = 0.44;
    ki = 0;
    kd = 1.5;
    dt = 1;
    r = 0;
    PIDTracerV2 *bananaPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
    commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
    calibrator->addPIDTracer(bananaPIDTracer);

    // PeachPIDTracerの初期化とCommandExecutorへの追加
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = carrotR;
    PIDTracerV2 *peachPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatePeach = new WheelDistancePredicate(peachDistance, robotAPI);
    calibrator->addPIDTracer(peachPIDTracer);
    commandExecutor->addCommand(peachPIDTracer, predicatePeach, GET_VARIABLE_NAME(peachPIDTracer));

    // OrangePIDTracerの初期化とCommandExecutorへの追加
    switch (orangePlan)
    {
    case 1:
    {
      // 強めのPDとI、なるべく弱めのRな走行
      orangeDistance = 72.5;
      pwm = 60;
      kp = 0.675;
      ki = 0.01;
      kd = kp * 3;
      dt = 1;
      r = -34;
      break;
    }
    case 2:
    {
      // TODO
      // 弱めのPDでRに頼った走行
      orangeDistance = 73.5; // TODO
      pwm = 60;
      kp = 0.4; // TODO
      ki = 0;
      kd = kp * 3;
      dt = 1;
      r = -34;
      break;
    }
    case 3:
    {
      // TODO
      orangeDistance = 72.5;
      pwm = 60;
      kp = 0.675;
      ki = 0.01;
      kd = kp * 3;
      dt = 1;
      r = -34;
      break;
    }
    }
    angle = 0; // TODO
    PIDTracerV2 *orangePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateOrange;
    if (useAnglePredicateAtOrange)
    {
      preicateOrange = new FacingRobotUseWheelPredicate(angle);
    }
    else
    {
      predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
    }
    calibrator->addPIDTracer(orangePIDTracer);
    commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

    if (facingAngleAtStarFruits)
    {
      angle = 10;
      PIDFacingAngleAbs *facingAngleC = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleC), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleC, facingAngleCPredicate, GET_VARIABLE_NAME(facingAngleC));
    }

    // StarFruitsWalkerの初期化とCommandExecutorへの追加
    pwm = 60;
    radius = 20;
    theta = -360; // 多めにしないと動かんのか？
    angle = 24;
    CurvatureWalkerCommandAndPredicate *starFuitsWalker = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
    Predicate *predicateStarFruits = new WheelDistancePredicate(starFruitsDistance, robotAPI);
    // Predicate *predicateStarFruits = new FacingRobotUseWheelPredicate(angle);
    // Predicate *predicateStarFruits = new GyroRotateAnglePredicate(angle);
    predicateStarFruits = predicateStarFruits->generateReversePredicate(); // 右車輪のほうが回転数多くなるのでそちらではかったほうが精度高くなりそう
    commandExecutor->addCommand(starFuitsWalker->getCommand(), predicateStarFruits, GET_VARIABLE_NAME(starFuitsWalker));
    float cherryPWM;
    float cherryKp;
    float cherryKi;
    float cherryKd;
    float cherryDt;
    float cherryR;
    float waterMelonPWM;
    float waterMelonKp;
    float waterMelonKi;
    float waterMelonKd;
    float waterMelonDt;
    float waterMelonR;

    switch (cherryPlan)
    {
    case 1:
    {
      // 強めのPD、なるべく弱めのRな走行
      cherryDistance = 60;
      cherryPWM = 65;
      cherryKp = 0.7;
      cherryKi = 0; // 0.025;
      cherryKd = 2.1;
      cherryDt = 1;
      cherryR = 38; // TODO
      break;
    }
    case 2:
    {
      // 弱めのPDでRに頼った走行
      cherryDistance = 60; // TODO
      cherryPWM = 65;
      cherryKp = 0.38; // TODO
      cherryKi = 0;
      cherryKd = cherryKp * 3; // TODO
      cherryDt = 1;
      cherryR = 31; // TODO
      break;
    }
    case 3:
    {
      // TODO
      cherryDistance = 60;
      cherryPWM = 65;
      cherryKp = 0.5;
      cherryKi = 0;
      cherryKd = cherryKp * 3;
      cherryDt = 1;
      cherryR = 28; // TODO
      break;
    }
    }

    switch (waterMelonPlan)
    {
    case 1:
    {
      // 強めのPDとI、なるべく弱めのRな走行
      waterMelonDistance = 273.5;
      waterMelonPWM = 65;
      waterMelonKp = 0.4; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 45; // TODO
      break;
    }
    case 2:
    {
      // 弱めのPDでRに頼った走行
      waterMelonDistance = 261;
      waterMelonPWM = 65;
      waterMelonKp = 0.4; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 33; // TODO
      break;
    }
    case 3:
    {
      // まあまあのPDでRに頼った走行
      waterMelonDistance = 264;
      waterMelonPWM = 65;
      waterMelonKp = 0.5; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 33; // TODO
      break;
    }
    case 4:
    {
      // 弱めのPIDでRに頼った走行
      waterMelonDistance = 263;
      waterMelonPWM = 65;
      waterMelonKp = 0.4;
      waterMelonKi = 0.01;
      waterMelonKd = waterMelonKp * 3;
      waterMelonDt = 1;
      waterMelonR = 29; // TODO
      break;
    }
    }

    pwm = cherryPWM;
    kp = cherryKp;
    ki = cherryKi;
    kd = cherryKd;
    dt = cherryDt;
    r = cherryR;
    PIDTracerV2 *cherryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCherry = new WheelDistancePredicate(cherryDistance, robotAPI);
    calibrator->addPIDTracer(cherryPIDTracer);
    commandExecutor->addCommand(cherryPIDTracer, predicateCherry, GET_VARIABLE_NAME(cherryPIDTracer));

    // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
    pwm = waterMelonPWM;
    kp = waterMelonKp;
    ki = waterMelonKi;
    kd = waterMelonKd;
    dt = waterMelonDt;
    r = waterMelonR;
    angle = 0; // TODO
    PIDTracerV2 *waterMelonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateWaterMelon;
    if (useAnglePredicateAtWaterMelon)
    {
      predicateWaterMelon = new FacingRobotUseWheelPredicate(angle);
    }
    else
    {
      predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
    }
    calibrator->addPIDTracer(waterMelonPIDTracer);
    commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

    if (facingAngleAtBokChoy)
    {
      angle = 330;
      PIDFacingAngleAbs *facingAngleC = new PIDFacingAngleAbs(facingAngleMode, angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleC), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleC, facingAngleCPredicate, GET_VARIABLE_NAME(facingAngleC));
    }

    // BokChoyWalkerの初期化とCommandExecutorへの追加
    pwm = 60;
    radius = 23.5;
    theta = -360; // 多めにしないと動かんのか？
    CurvatureWalkerCommandAndPredicate *bokChoyWalker = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
    predicateBokChoy = predicateBokChoy->generateReversePredicate(); // 右車輪のほうが回転数多くなるのでそちらではかったほうが精度高くなりそう
    commandExecutor->addCommand(bokChoyWalker->getCommand(), predicateBokChoy, GET_VARIABLE_NAME(bokChoyWalker));

    float dorianPWM;
    float dorianKp;
    float dorianKi;
    float dorianKd;
    float dorianDt;
    float dorianR;

    // DorianPIDTracerの初期化とCommandExecutorへの追加
    switch (dorianPlan)
    {
    case 1:
    {
      // まぁまぁなPID走行
      dorianPWM = 35;
      dorianKp = 0.58;
      dorianKi = 0.006;
      dorianKd = kp * 3;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    case 2:
    {
      // 弱めのPD走行
      dorianPWM = 35;
      dorianKp = 0.48;
      dorianKi = 0;
      dorianKd = kp * 3;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    case 3:
    {
      // 強めのPD走行
      dorianPWM = 35;
      dorianKp = 0.5;
      dorianKi = 0;
      dorianKd = kp * 3;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    case 4:
    {
      // ばななを使う
      dorianPWM = 45;
      dorianKp = 0.44;
      dorianKi = 0;
      dorianKd = 1.5;
      dorianDt = 1;
      dorianR = 0;
      break;
    }
    }
    pwm = dorianPWM;
    kp = dorianKp;
    ki = dorianKi;
    kd = dorianKd;
    dt = dorianDt;
    r = dorianR;
    PIDTracerV2 *dorianPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateDorian = new WheelDistancePredicate(dorianDistance, robotAPI);
    calibrator->addPIDTracer(dorianPIDTracer);
    commandExecutor->addCommand(dorianPIDTracer, predicateDorian, GET_VARIABLE_NAME(dorianPIDTracer));

    // HassakuPIDTracerの初期化とCommandExecutorへの追加
    pwm = 25;
    kp = 0.64;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *hassakuPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateHassaku = new WheelDistancePredicate(hassakuDistance, robotAPI);
    calibrator->addPIDTracer(hassakuPIDTracer);
    commandExecutor->addCommand(hassakuPIDTracer, predicateHassaku, GET_VARIABLE_NAME(hassakuPIDTracer));

    // RadishPIDTracerの初期化とCommandExecutorへの追加
    pwm = dorianPWM;
    kp = dorianKp;
    ki = dorianKi;
    kd = dorianKd;
    dt = dorianDt;
    r = dorianR;
    PIDTracerV2 *radishPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateRadish = new WheelDistancePredicate(radishDistance, robotAPI);
    calibrator->addPIDTracer(radishPIDTracer);
    commandExecutor->addCommand(radishPIDTracer, predicateRadish, GET_VARIABLE_NAME(radishPIDTracer));

    // MelonPIDTracerの初期化とCommandExecutorへの追加  ここから
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = carrotR;
    PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
    calibrator->addPIDTracer(melonPIDTracer);
    commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

    // nutsの初期化とCommandExecutorへの追加  ここから
    pwm = 50;
    kp = 0.44;
    ki = 0.001;
    kd = 1.5;
    dt = 1;
    r = 0;
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *nutsPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatenuts = new WheelDistancePredicate(nutsDistance, robotAPI);
    calibrator->addPIDTracer(nutsPIDTracer);
    commandExecutor->addCommand(nutsPIDTracer, predicatenuts, GET_VARIABLE_NAME(nutsPIDTracer));

    // LemonPIDTracerの初期化とCommandExecutorへの追加
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = carrotKd;
    dt = carrotDt;
    r = carrotR;

    PIDTracerV2 *lemonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateLemon = new WheelDistancePredicate(lemonDistance, robotAPI);
    calibrator->addPIDTracer(lemonPIDTracer);
    commandExecutor->addCommand(lemonPIDTracer, predicateLemon, GET_VARIABLE_NAME(lemonPIDTracer));

    // CucumberPIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.5;
    ki = 0.001;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *cucumberPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
    calibrator->addPIDTracer(cucumberPIDTracer);
    commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

    // StrawberryPIDTracerの初期化とCommandExecutorへの追加
    pwm = carrotPWM;
    kp = carrotKp;
    ki = carrotKi;
    kd = kp; // TODO 試して
    dt = carrotDt;
    r = -44;
    PIDTracerV2 *strawberryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
    predicateStrawberry = predicateStrawberry->generateReversePredicate();
    calibrator->addPIDTracer(strawberryPIDTracer);
    commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

    // CabbagePIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.5;
    ki = 0.001;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *cabbagePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCabbage = new WheelDistancePredicate(cabbageDistance, robotAPI);
    calibrator->addPIDTracer(cabbagePIDTracer);
    commandExecutor->addCommand(cabbagePIDTracer, predicateCabbage, GET_VARIABLE_NAME(cabbagePIDTracer));
    // Commandの定義とCommandExecutorへの追加ここまで

    ResetPWMCoefficient *resetPWMCoefficient = new ResetPWMCoefficient();
    commandExecutor->addCommand(resetPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(resetPWMCoefficient));

#if defined(SimulatorMode) | defined(DisableCalibration)
    // シミュレータはPIDTargetBrightnessをキャリブレーションしないので値を設定する必要がある
    carrotPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    bananaPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    peachPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    orangePIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    waterMelonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    dorianPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    radishPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    melonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    lemonPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    cucumberPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    strawberryPIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
    cabbagePIDTracer->setTargetBrightness(blackWhiteEdgeTargetBrightness);
#endif
  }
// ↑ここまで実方↑
#endif

  // ↓ここから実方↓
  if (true) // スラローム
  {
    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;
    float pidR;
    float straightKp = 0.05;
    float straightKi = 0;
    float straightKd = 0.05;
    float straightDt = 1;
    float faKp = 0.7;
    float faKi = 0.01;
    float faKd = 0.7;
    float faDt = 1;

    commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "stopper");
    resetArmAngle = new ResetArmAngle();
    commandExecutor->addCommand(resetArmAngle, new FinishedCommandPredicate(resetArmAngle), GET_VARIABLE_NAME(resetArmAngle));

    // ガレージカードの色取得用ColorReader
    float slalomAngleOffset = 0;

    float coefficientPWM;
    float coefficientPWMForCurve;

    float radius;
    float theta;

    float angle;
    float distance;

    float leftPWM;
    float rightPWM;

    int numberOfTime;
    uint64_t waitFaUsec = 1000000;

    FacingAngleMode facingAngleMode = FA_WheelCount;

#ifdef SimulatorMode
    coefficientPWM = 2;
    coefficientPWMForCurve = 2;
#else
    coefficientPWM = 1;
    coefficientPWMForCurve = 1;
#endif

    Stopper *stopper = new Stopper();

#ifdef SimulatorMode
    pwm = 30 * coefficientPWM;
    kp = 0.7;
    ki = 0.2;
    kd = 0.7;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *pidTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
    pwm = 20 * coefficientPWM;
    ColorPIDTracerV2 *lowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
    pwm = 10 * coefficientPWM;
    ColorPIDTracerV2 *verryLowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
#else
    pwm = 30 * coefficientPWM;
    kp = 0.155;
    ki = 0.001;
    kd = 0.572;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *pidTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);

    pwm = 20 * coefficientPWM;
    kp = 0.195;
    ki = 0;
    kd = 0.39;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *lowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);

    pwm = 10 * coefficientPWM;
    kp = 0.305;
    ki = 0;
    kd = 0.522;
    dt = 1;
    pidR = 0;
    ColorPIDTracerV2 *verryLowPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
#endif
    calibrator->addColorPIDTracer(pidTracer);
    calibrator->addColorPIDTracer(lowPWMTracer);
    calibrator->addColorPIDTracer(verryLowPWMTracer);
#ifdef SimulatorMode
    float targetBrightness = 20;
    rgb_raw_t targetRGB;
    targetRGB.r = blackWhiteEdgeR;
    targetRGB.g = 60;
    targetRGB.b = 60;
    pidTracer->setTargetColor(targetRGB);
    lowPWMTracer->setTargetColor(targetRGB);
#endif

    // スラローム進入ここから
    {
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

      /*
      // スラローム位置補正。アームを下げたまま直進。
      numberOfTime = 65;
      leftPWM = 7;
      rightPWM = 7;
      Walker *lowWalker = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
      commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");
      */
      numberOfTime = 35;
      leftPWM = 12;
      rightPWM = 12;
      Walker *lowWalker0 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker0, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker0));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = 12;
      rightPWM = -4;
      Walker *lowWalker1 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker1, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker1));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = -4;
      rightPWM = 12;
      Walker *lowWalker2 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker2, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker2));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = 7;
      rightPWM = -2;
      Walker *lowWalker3 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker3, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker3));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 20;
      leftPWM = -2;
      rightPWM = 7;
      Walker *lowWalker4 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker4, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker4));
      // commandExecutor->addCommand(new ReleaseWheel(), new NumberOfTimesPredicate(15), "releaseWheel");

      numberOfTime = 35;
      leftPWM = 7;
      rightPWM = 7;
      Walker *lowWalker5 = new Walker(leftPWM, rightPWM);
      commandExecutor->addCommand(lowWalker5, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker5));
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
      pwm = -5 * coefficientPWM;
      distance = -4;
      PIDStraightWalker *back = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
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
      distance = 27;
      pwm = 30;
      PIDStraightWalker *walker1_y = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walker1_y->setTargetDifferenceWheelCount(0);
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
    }
    // スラローム進入ここまで

    // スラローム位置補正ここから
    {
      // ジャイロで向き調節
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
      angle = -90;
      PIDFacingAngleAbs *facingAngleX = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleXPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleX), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleX, facingAngleXPredicate, GET_VARIABLE_NAME(facingAngleX));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 白を拾うまで直進
      pwm = 5;
      pwm = 7;
      PIDStraightWalker *walkerW = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walkerW->setTargetDifferenceWheelCount(0);
      RawColorPredicate *whitePredicate = new WhiteAtSlaromPredicate();
      commandExecutor->addCommand(walkerW, whitePredicate, GET_VARIABLE_NAME(walkerW));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // バック
      pwm = -6;
      pwm = -10;
      distance = -4.2;
      PIDStraightWalker *walkerB = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walkerB->setTargetDifferenceWheelCount(0);
      WheelDistancePredicate *walkerBPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walkerB, walkerBPredicate, GET_VARIABLE_NAME(walkerB));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      angle = 0;
      PIDFacingAngleAbs *facingAngleC = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleC), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleC, facingAngleCPredicate, GET_VARIABLE_NAME(facingAngleC));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      leftPWM = 8;
      rightPWM = 8;
      Walker *walkerC2 = new Walker(leftPWM, rightPWM);
      Predicate *walkerC2Predicate = new WheelDistancePredicate(6, robotAPI);
      commandExecutor->addCommand(walkerC2, walkerC2Predicate, GET_VARIABLE_NAME(walkerC2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

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

      angle = 0;
      FacingAngleAbs *facingAngleC3 = new FacingAngleAbs(facingAngleMode, pwm, slalomAngleOffset + angle);
      commandExecutor->addCommand(facingAngleC3, new FinishedCommandPredicate(facingAngleC3), GET_VARIABLE_NAME(facingAngleC3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */
    }
    // スラローム位置補正ここまで

    // 指示待ち走行ここから
    {
      // 向き調節
      angle = 0;
      PIDFacingAngleAbs *facingAngle1 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle1Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle1), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle1, facingAngle1Predicate, GET_VARIABLE_NAME(facingAngle1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 位置調節
      pwm = 8 * coefficientPWM;
      pwm = 10 * coefficientPWM;
      distance = 10;
      HedgehogUsePID *headgehogA = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehogA, new FinishedCommandPredicate(headgehogA), GET_VARIABLE_NAME(headgehogA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      pwm = 15 * coefficientPWMForCurve;
      radius = 14;
      theta = 50;
      CurvatureWalkerCommandAndPredicate *curve1 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve1->getCommand(), curve1->getPredicate(), GET_VARIABLE_NAME(curve1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 7 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      pwm = 15 * coefficientPWMForCurve;
      radius = 14;
      theta = -50;
      CurvatureWalkerCommandAndPredicate *curve2 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve2->getCommand(), curve2->getPredicate(), GET_VARIABLE_NAME(curve2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      pwm = 10 * coefficientPWMForCurve;
      angle = 0;
      PIDFacingAngleAbs *facingAngle3 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle3Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle3), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle3, facingAngle3Predicate, GET_VARIABLE_NAME(facingAngle3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 直進
      pwm = 7;
      pwm = 7;
      distance = 2;
      PIDStraightWalker *walkerA = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      WheelDistancePredicate *walkerAPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walkerA, walkerAPredicate, GET_VARIABLE_NAME(walkerA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 位置調節
      pwm = 7 * coefficientPWM;
      distance = 8;
      Hedgehog *headgehog1 = new Hedgehog(distance, pwm);
      commandExecutor->addCommand(headgehog1, new FinishedCommandPredicate(headgehog1), GET_VARIABLE_NAME(headgehog1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      radius = 12.5;
      theta = -45;
      CurvatureWalkerCommandAndPredicate *curve3 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve3->getCommand(), curve3->getPredicate(), GET_VARIABLE_NAME(curve3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      distance = 1.5;
      pwm = 10;
      PIDStraightWalker *walkerD = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      WheelDistancePredicate *walkerDPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walkerD, walkerDPredicate, GET_VARIABLE_NAME(walkerD));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      /*
      // 向き調節
      angle = -45;
      PIDFacingAngleAbs *facingAngle4 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle4Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle4), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle4, facingAngle4Predicate, GET_VARIABLE_NAME(facingAngle4));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 7 * coefficientPWMForCurve;
      pwm = 10 * coefficientPWMForCurve;
      radius = 12.5;
      theta = 45;
      CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

/*
// 直進位置調節
pwm = -10 * coefficientPWM;
distance = -3;
HedgehogUsePID *headgehogZ = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
commandExecutor->addCommand(headgehogZ, new FinishedCommandPredicate(headgehogZ), GET_VARIABLE_NAME(headgehogZ));
commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
*/

// 直進
#ifdef SlalomPattern1
      distance = 8.5;
      pwm = 10;
      PIDStraightWalker *walker5 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
#else
      distance = 4;
      pwm = 10;
      PIDStraightWalker *walker5 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
#endif

      // 向き調節
      angle = 0;
      PIDFacingAngleAbs *facingAngle42 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle42Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle42), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle42, facingAngle42Predicate, GET_VARIABLE_NAME(facingAngle42));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 8 * coefficientPWMForCurve;
      radius = 11.2; // 11.5;
      theta = 50;
      CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 5 * coefficientPWMForCurve;
      radius = 11.2; // 11.5;
      theta = -50;
      CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      PIDFacingAngleAbs *facingAngle5 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset, faKp, faKi, faKd, faDt);
      Predicate *facingAngle5Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle5), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle5, facingAngle5Predicate, GET_VARIABLE_NAME(facingAngle5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 直進位置調節
      int diff = 2;
      // pwm = 10 * coefficientPWM;
      pwm = 7 * coefficientPWM;
      distance = 3 + diff;
      HedgehogUsePID *headgehog2 = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehog2, new FinishedCommandPredicate(headgehog2), GET_VARIABLE_NAME(headgehog2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      angle = 23.5;
      PIDFacingAngleAbs *facingAngleCo1 = new PIDFacingAngleAbs(facingAngleMode, angle + slalomAngleOffset, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCo1Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngleCo1), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleCo1, facingAngleCo1Predicate, GET_VARIABLE_NAME(facingAngleCo1));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 色取得
      commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));

      // 向き調節
      PIDFacingAngleAbs *facingAngleCo2 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset, faKp, faKi, faKd, faDt);
      Predicate *facingAngleCo2Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngleCo2), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleCo2, facingAngleCo2Predicate, GET_VARIABLE_NAME(facingAngleCo2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

#ifdef SlalomPattern1
      // 直進位置調節
      pwm = 10 * coefficientPWM;
      pwm = 7;
      // pwm = 20 * coefficientPWM;
      distance = 12 + diff;
      HedgehogUsePID *headgehogAA = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehogAA, new FinishedCommandPredicate(headgehogAA), GET_VARIABLE_NAME(headgehogAA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      radius = 12.5;
      theta = -87.5;
      CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 90度左を向く
      angle = -90;
      PIDFacingAngleAbs *facingAngleY = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngleYPredicate = new ORPredicate(new FinishedCommandPredicate(facingAngleY), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngleY, facingAngleYPredicate, GET_VARIABLE_NAME(facingAngleY));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 位置調節
      pwm = 6 * coefficientPWM;
      // pwm = 10 * coefficientPWM;
      pwm = 7;
      distance = 5;
      HedgehogUsePID *headgehog3 = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehog3, new FinishedCommandPredicate(headgehog3), GET_VARIABLE_NAME(headgehog3));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 直進
      distance = 2.5;
      pwm = 6;
      pwm = 10;
      PIDStraightWalker *walker7 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 150度左を向く
      angle = -150;
      PIDFacingAngleAbs *facingAngle8 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle8Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle8), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle8, facingAngle8Predicate, GET_VARIABLE_NAME(facingAngle8));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 37;
      theta = 70;
      CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 30度左を向く
      angle = -30;
      radius = 18;
      PIDFacingAngleAbs *facingAngle9 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle9Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle9), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle9, facingAngle9Predicate, GET_VARIABLE_NAME(facingAngle9));
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
      // 直進位置調節
      pwm = 10 * coefficientPWM;
      pwm = 7;
      // pwm = 20 * coefficientPWM;
      distance = 16 + diff;
      HedgehogUsePID *headgehogAA = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehogAA, new FinishedCommandPredicate(headgehogAA), GET_VARIABLE_NAME(headgehogAA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 10;
      theta = -40;
      CurvatureWalkerCommandAndPredicate *curveD = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curveD->getCommand(), curveD->getPredicate(), GET_VARIABLE_NAME(curveD));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 115度左を向く
      angle = -122.5;
      PIDFacingAngleAbs *facingAngle7 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle7Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle7), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle7, facingAngle7Predicate, GET_VARIABLE_NAME(facingAngle7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      distance = 5;
      pwm = 8;
      PIDStraightWalker *walker7 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walker7->setTargetDifferenceWheelCount(0);
      WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 43;
      theta = 21;
      CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 90度左を向く
      angle = -90;
      PIDFacingAngleAbs *facingAngle8 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle8Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle8), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle8, facingAngle8Predicate, GET_VARIABLE_NAME(facingAngle8));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      leftPWM = 10 * coefficientPWM;
      rightPWM = 10 * coefficientPWM;
      distance = 5;
      Walker *walker8y = new Walker(leftPWM, rightPWM);
      WheelDistancePredicate *walker8yPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker8y, walker8yPredicate, GET_VARIABLE_NAME(walker8y));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 15;
      theta = 50;
      CurvatureWalkerCommandAndPredicate *curve7 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve7->getCommand(), curve7->getPredicate(), GET_VARIABLE_NAME(curve7));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      pwm = 20 * coefficientPWMForCurve;
      radius = 18;
      theta = -30;
      CurvatureWalkerCommandAndPredicate *curve8 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve8->getCommand(), curve8->getPredicate(), GET_VARIABLE_NAME(curve8));

      // カーブ
      pwm = 10 * coefficientPWMForCurve;
      radius = 18;
      theta = -360;
      CurvatureWalkerCommandAndPredicate *curve9 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curve9->getCommand(), new BlackPredicate(), GET_VARIABLE_NAME(curve9));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      /*
      // 直進
      distance = 15;
      pwm = 10;
      PIDStraightWalker *walker9 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      walker9->setTargetDifferenceWheelCount(0);
      WheelDistancePredicate *walker9Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker9, walker9Predicate, GET_VARIABLE_NAME(walker9));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      /*
      // 110度左を向く
      angle = -150;
      PIDFacingAngleAbs *facingAngle10 = new PIDFacingAngleAbs(facingAngleMode, slalomAngleOffset + angle, faKp, faKi, faKd, faDt);
      Predicate *facingAngle10Predicate = new ORPredicate(new FinishedCommandPredicate(facingAngle10), new TimerPredicate(waitFaUsec));
      commandExecutor->addCommand(facingAngle10, facingAngle10Predicate, GET_VARIABLE_NAME(facingAngle10));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      /*
      // 黒線まで直進する
      pwm = 10;
      PIDStraightWalker *walkerO = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *blackPredicate = new BlackPredicate();
      commandExecutor->addCommand(walkerO, blackPredicate, GET_VARIABLE_NAME(walkerO));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
      */

      // 青線までPIDトレースする
      RawColorPredicate *blueEdgePredicate = new BlueEdgePredicate();
      commandExecutor->addCommand(verryLowPWMTracer, blueEdgePredicate, GET_VARIABLE_NAME(lowPWMTracer));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
#endif
    }
    // 指示待ち走行ここまで
  }
  // ↑ここまで実方↑

  // ↓ここから小路↓
  if (true) // ガレージ
  {
    float pwm = 10;
    float kp = 0.23;
    float ki = 0;
    float kd = 0.5;
    float dt = 1;
    float pidR = 0;
    ColorPIDTracerV2 *colorPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
    calibrator->addColorPIDTracer(colorPWMTracer);
    /*
    RawColorPredicate *blueEdgePredicate = new BlueEdgePredicate();
    //青ラインまで進む
    commandExecutor->addCommand(colorPWMTracer, blueEdgePredicate, GET_VARIABLE_NAME(colorPWMTracer));
    */
    // 停止コマンドの初期化とCommandExecutorへの追加
    Stopper *stopper = new Stopper();
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
    float leftPow;
    float rightPow;
    leftPow = -15;
    rightPow = -15;
    Walker *walker1 = new Walker(leftPow, rightPow);
    Predicate *predicate1 = new WheelDistancePredicate(-19, robotAPI);
    commandExecutor->addCommand(walker1, predicate1, GET_VARIABLE_NAME(walker1));
    Stopper *stopper1 = new Stopper();
    Predicate *predicateS1 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper1, predicateS1, GET_VARIABLE_NAME(stoppper1));
    // 2,90ど左回転
    leftPow = -10;
    rightPow = 10;
    Walker *walker2 = new Walker(leftPow, rightPow);
    Predicate *predicate2 = new WheelDistancePredicate(-13, robotAPI);
    commandExecutor->addCommand(walker2, predicate2, GET_VARIABLE_NAME(walker2));
    // CommandAndPredicate *predicate2 = new RotateRobotUseGyroCommandAndPredicate(-91, 20, robotAPI);
    // commandExecutor->addCommand(predicate2->getCommand(), predicate2->getPredicate(), GET_VARIABLE_NAME(predicate2->getCommand()));
    Stopper *stopper2 = new Stopper();
    Predicate *predicateS2 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper2, predicateS2, GET_VARIABLE_NAME(stoppper2));
    // 3「黒検知するまで」直進
    leftPow = 40;
    rightPow = 40;
    Walker *walker3 = new Walker(leftPow, rightPow);
    Predicate *predicate3 = new WheelDistancePredicate(50, robotAPI);
    commandExecutor->addCommand(walker3, predicate3, GET_VARIABLE_NAME(walker3));
    Predicate *predicate3b = new ColorPredicate(COLOR_BLACK);
    commandExecutor->addCommand(walker3, predicate3b, GET_VARIABLE_NAME(walker3));
    // 4, 少し下がって運搬物中心に向く
    // Walker *walker4 = new Walker(-10, -10);
    // Predicate *predicate4b = new WheelDistancePredicate(-5, robotAPI);
    // commandExecutor->addCommand(walker4, predicate4b, GET_VARIABLE_NAME(walker4));
    leftPow = 10;
    rightPow = -10;
    CommandAndPredicate *predicate4 = new RotateRobotUseGyroCommandAndPredicate(115, 20, robotAPI);
    commandExecutor->addCommand(predicate4->getCommand(), predicate4->getPredicate(), GET_VARIABLE_NAME(predicate4->getCommand()));
    Stopper *stopper4 = new Stopper();
    Predicate *predicateS4 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper4, predicateS4, GET_VARIABLE_NAME(stoppper4));
    // 8,灰色で止まり、運搬物中心に向かって直進
    // tyokusin
    Command *walker = new Walker(10, 10);
    int *r = new int(25);
    int *g = new int(30);
    int *b = new int(40);
    // Predicate *predicate8 = new WheelDistancePredicate(1, robotAPI);
    //   int *r = new int(75);
    // int *g = new int(75);
    // int *b = new int(105);
    // PID
    //  pwm = 20;
    //    kp = 0.7;
    //    ki = 0.2;
    //    kd = 0.7;
    //    dt = 1;
    //    PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
    //    Predicate *predicateBanana = new WheelDistancePredicate(20, robotAPI);
    //    commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
    //    calibrator->addPIDTracer(bananaPIDTracer);
    Predicate *predicateCarrot = new WheelDistancePredicate(10, robotAPI);
    commandExecutor->addCommand(colorPWMTracer, predicateCarrot, GET_VARIABLE_NAME(colorPWMTracer));
    // Predicate *predicate8 = new RawColorPredicate(r, BETWEEN15, g, BETWEEN15, b, BETWEEN15);
    Predicate *predicate8 = new GrayPredicate();
    // commandExecutor->addCommand(walker, new ColorPredicate(COLOR_BLACK), GET_VARIABLE_NAME(walker));
    commandExecutor->addCommand(walker, predicate8, GET_VARIABLE_NAME(walker));
    Stopper *stopper8 = new Stopper();
    Predicate *predicateS8 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
    CommandAndPredicate *predicate8r = new RotateRobotUseGyroCommandAndPredicate(-20, 10, robotAPI);
    commandExecutor->addCommand(predicate8r->getCommand(), predicate8r->getPredicate(), GET_VARIABLE_NAME(predicate8r->getCommand()));
    commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
    leftPow = 20;
    rightPow = 20;
    Walker *walker8 = new Walker(leftPow, rightPow);
    Predicate *predicate8w = new WheelDistancePredicate(11, robotAPI);
    commandExecutor->addCommand(walker8, predicate8w, GET_VARIABLE_NAME(walker8));
    commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
    // 9，左に旋回する。
    leftPow = -5;
    rightPow = 5;
    CommandAndPredicate *predicate9 = new RotateRobotUseGyroCommandAndPredicate(-5, 10, robotAPI);
    commandExecutor->addCommand(predicate9->getCommand(), predicate9->getPredicate(), GET_VARIABLE_NAME(predicate9->getCommand()));
    Stopper *stopper9 = new Stopper();
    Predicate *predicateS9 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper9, predicateS9, GET_VARIABLE_NAME(stoppper9));
    // 10,青丸に向かって直進
    leftPow = 20;
    rightPow = 20;
    Walker *walker10 = new Walker(leftPow, rightPow);
    Predicate *predicate10 = new WheelDistancePredicate(15, robotAPI);
    commandExecutor->addCommand(walker10, predicate10, GET_VARIABLE_NAME(walker10));
    commandExecutor->addCommand(colorPWMTracer, predicate10, GET_VARIABLE_NAME(colorPWMTracer));
    commandExecutor->addCommand(colorPWMTracer, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(colorPWMTracer));
    //青の後ちょっと進む
    //  Predicate *predicate10bl = new WheelDistancePredicate(10, robotAPI);
    //   commandExecutor->addCommand(walker10, predicate10bl, GET_VARIABLE_NAME(walker10));
    Stopper *stopper10 = new Stopper();
    Predicate *predicateS10 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper10, predicateS10, GET_VARIABLE_NAME(stoppper10));
    // １１ 指定角度右回転（青ラインに向く）
    CommandAndPredicate *predicate11 = new RotateRobotUseGyroCommandAndPredicate(4, 5, robotAPI);
    commandExecutor->addCommand(predicate11->getCommand(), predicate11->getPredicate(), GET_VARIABLE_NAME(predicate11->getCommand()));
    Stopper *stopper11 = new Stopper();
    Predicate *predicateS11 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    // 12「青検知するまで」直進
    leftPow = 20;
    rightPow = 20;
    Walker *walker12 = new Walker(leftPow, rightPow);
    Predicate *predicate12 = new WheelDistancePredicate(27, robotAPI);
    commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
    Walker *walker12b = new Walker(leftPow, rightPow);
    commandExecutor->addCommand(walker12b, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(walker12b));
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    bool useFacingAngle0 = false;
    if (useFacingAngle0) // FacingAngleによるガレージイン
    {
      float straightKp = 1;
      float straightKi = 0;
      float straightKd = 1;
      float straightDt = 1;
      float pwm = 20;
      float distance = 8;
      PIDStraightWalker *straightWalker = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *straightWalkerPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(straightWalker, straightWalkerPredicate, GET_VARIABLE_NAME(straightWalker));
      float faKp = 0.3;
      float faKi = 0.015;
      float faKd = 0.7;
      float faDt = 1;
      float waitFaUsec = 2000000;
      float angle = 0;
      PIDFacingAngleAbs *facing0 = new PIDFacingAngleAbs(FA_WheelCount, angle, faKp, faKi, faKd, faDt);
      commandExecutor->addCommand(facing0, new ORPredicate(new TimerPredicate(waitFaUsec), new FinishedCommandPredicate(facing0)), GET_VARIABLE_NAME(facing0));
    }
    else // 青トレースによるガレージイン
    {
      // 13,10ど右回転 ８回
      bool gyro = false;
      if (gyro == true)
      { //ジャイロで調節するならtrueに
        CommandAndPredicate *predicate13 = new RotateRobotUseGyroCommandAndPredicate(40, 5, robotAPI);
        Walker *walker13S = new Walker(leftPow, rightPow);
        Predicate *predicate13S = new WheelDistancePredicate(1, robotAPI);
        for (int i = 0; i < 2; i++)
        {
          commandExecutor->addCommand(predicate13->getCommand(), predicate13->getPredicate(), GET_VARIABLE_NAME(predicate13->getCommand()));
          commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
          commandExecutor->addCommand(walker13S, predicate13S, GET_VARIABLE_NAME(walker13S));
          commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
        }
      }
      else
      {
        leftPow = 10;
        rightPow = -10;
        Walker *walker2 = new Walker(leftPow, rightPow);
        Predicate *predicate2 = new WheelDistancePredicate(10, robotAPI);
        commandExecutor->addCommand(walker2, predicate2, GET_VARIABLE_NAME(walker2));
      }
      ColorPIDTracerV2 *bluePWMTracer = new ColorPIDTracerV2(LEFT_TRACE, Trace_R, 5, kp, ki, kd, dt, pidR);
      calibrator->addColorPIDTracer(bluePWMTracer);
      Predicate *predicate13blue = new WheelDistancePredicate(10, robotAPI);
      commandExecutor->addCommand(bluePWMTracer, predicate13blue, GET_VARIABLE_NAME(bluePWMTracer));
      commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    }
    // 14,ガレージに直進（取得した色ごとに分岐させる）colorReadergetColor()で色を取得できる
    Command *dealingWithGarage14 = new DealingWithGarage(garageCardColorPtr, commandExecutor, false);
    Predicate *predicate14 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(dealingWithGarage14, predicate14, GET_VARIABLE_NAME(dealingWithGarage14));
  }
  // ↑ここまで小路↑

  commandExecutor->addCommand(new Stopper(), new NumberOfTimesPredicate(1), "stopper");
}
#endif

#ifdef GoalSanekataScenarioTestMode
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

#ifdef PIDFacingAngleAbsTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  FacingAngleMode mode = FA_WheelCount;
  float angle = 90;
  float kp = 0.7;
  float ki = 0.1;
  float kd = 0.7;
  float dt = 1;
  float waitFaUsec = 1000000;

  PIDFacingAngleAbs *pidFacingAngleAbs = new PIDFacingAngleAbs(mode, angle, kp, ki, kd, dt);
  Predicate *facingAnglePredicate = new ORPredicate(new FinishedCommandPredicate(pidFacingAngleAbs), new TimerPredicate(waitFaUsec));
  commandExecutor->addCommand(pidFacingAngleAbs, facingAnglePredicate, GET_VARIABLE_NAME(facingAngleX));
}
#endif

#ifdef GarageKomichiMode1
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
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();
  // colorid_t *garageCardColorPtr = new colorid_t(COLOR_RED);

  {
    float pwm = 10;
    float kp = 0.23;
    float ki = 0;
    float kd = 0.5;
    float dt = 1;
    float pidR = 0;
    ColorPIDTracerV2 *colorPWMTracer = new ColorPIDTracerV2(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt, pidR);
    calibrator->addColorPIDTracer(colorPWMTracer);
    /*
    RawColorPredicate *blueEdgePredicate = new BlueEdgePredicate();
    //青ラインまで進む
    commandExecutor->addCommand(colorPWMTracer, blueEdgePredicate, GET_VARIABLE_NAME(colorPWMTracer));
    */
    // 停止コマンドの初期化とCommandExecutorへの追加
    Stopper *stopper = new Stopper();
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
    float leftPow;
    float rightPow;
    leftPow = -15;
    rightPow = -15;
    Walker *walker1 = new Walker(leftPow, rightPow);
    Predicate *predicate1 = new WheelDistancePredicate(-19, robotAPI);
    commandExecutor->addCommand(walker1, predicate1, GET_VARIABLE_NAME(walker1));
    Stopper *stopper1 = new Stopper();
    Predicate *predicateS1 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper1, predicateS1, GET_VARIABLE_NAME(stoppper1));
    // 2,90ど左回転
    leftPow = -10;
    rightPow = 10;
    Walker *walker2 = new Walker(leftPow, rightPow);
    Predicate *predicate2 = new WheelDistancePredicate(-13, robotAPI);
    commandExecutor->addCommand(walker2, predicate2, GET_VARIABLE_NAME(walker2));
    // CommandAndPredicate *predicate2 = new RotateRobotUseGyroCommandAndPredicate(-91, 20, robotAPI);
    // commandExecutor->addCommand(predicate2->getCommand(), predicate2->getPredicate(), GET_VARIABLE_NAME(predicate2->getCommand()));
    Stopper *stopper2 = new Stopper();
    Predicate *predicateS2 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper2, predicateS2, GET_VARIABLE_NAME(stoppper2));
    // 3「黒検知するまで」直進
    leftPow = 40;
    rightPow = 40;
    Walker *walker3 = new Walker(leftPow, rightPow);
    Predicate *predicate3 = new WheelDistancePredicate(50, robotAPI);
    commandExecutor->addCommand(walker3, predicate3, GET_VARIABLE_NAME(walker3));
    Predicate *predicate3b = new ColorPredicate(COLOR_BLACK);
    commandExecutor->addCommand(walker3, predicate3b, GET_VARIABLE_NAME(walker3));
    // 4, 少し下がって運搬物中心に向く
    // Walker *walker4 = new Walker(-10, -10);
    // Predicate *predicate4b = new WheelDistancePredicate(-5, robotAPI);
    // commandExecutor->addCommand(walker4, predicate4b, GET_VARIABLE_NAME(walker4));
    leftPow = 10;
    rightPow = -10;
    CommandAndPredicate *predicate4 = new RotateRobotUseGyroCommandAndPredicate(115, 20, robotAPI);
    commandExecutor->addCommand(predicate4->getCommand(), predicate4->getPredicate(), GET_VARIABLE_NAME(predicate4->getCommand()));
    Stopper *stopper4 = new Stopper();
    Predicate *predicateS4 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper4, predicateS4, GET_VARIABLE_NAME(stoppper4));
    // 8,灰色で止まり、運搬物中心に向かって直進
    // tyokusin
    Command *walker = new Walker(10, 10);
    int *r = new int(25);
    int *g = new int(30);
    int *b = new int(40);
    // Predicate *predicate8 = new WheelDistancePredicate(1, robotAPI);
    //   int *r = new int(75);
    // int *g = new int(75);
    // int *b = new int(105);
    // PID
    //  pwm = 20;
    //    kp = 0.7;
    //    ki = 0.2;
    //    kd = 0.7;
    //    dt = 1;
    //    PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
    //    Predicate *predicateBanana = new WheelDistancePredicate(20, robotAPI);
    //    commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
    //    calibrator->addPIDTracer(bananaPIDTracer);
    Predicate *predicateCarrot = new WheelDistancePredicate(10, robotAPI);
    commandExecutor->addCommand(colorPWMTracer, predicateCarrot, GET_VARIABLE_NAME(colorPWMTracer));
    // Predicate *predicate8 = new RawColorPredicate(r, BETWEEN15, g, BETWEEN15, b, BETWEEN15);
    Predicate *predicate8 = new GrayPredicate();
    // commandExecutor->addCommand(walker, new ColorPredicate(COLOR_BLACK), GET_VARIABLE_NAME(walker));
    commandExecutor->addCommand(walker, predicate8, GET_VARIABLE_NAME(walker));
    Stopper *stopper8 = new Stopper();
    Predicate *predicateS8 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
    CommandAndPredicate *predicate8r = new RotateRobotUseGyroCommandAndPredicate(-20, 10, robotAPI);
    commandExecutor->addCommand(predicate8r->getCommand(), predicate8r->getPredicate(), GET_VARIABLE_NAME(predicate8r->getCommand()));
    commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
    leftPow = 20;
    rightPow = 20;
    Walker *walker8 = new Walker(leftPow, rightPow);
    Predicate *predicate8w = new WheelDistancePredicate(11, robotAPI);
    commandExecutor->addCommand(walker8, predicate8w, GET_VARIABLE_NAME(walker8));
    commandExecutor->addCommand(stopper8, predicateS8, GET_VARIABLE_NAME(stoppper8));
    // 9，左に旋回する。
    leftPow = -5;
    rightPow = 5;
    CommandAndPredicate *predicate9 = new RotateRobotUseGyroCommandAndPredicate(-5, 10, robotAPI);
    commandExecutor->addCommand(predicate9->getCommand(), predicate9->getPredicate(), GET_VARIABLE_NAME(predicate9->getCommand()));
    Stopper *stopper9 = new Stopper();
    Predicate *predicateS9 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper9, predicateS9, GET_VARIABLE_NAME(stoppper9));
    // 10,青丸に向かって直進
    leftPow = 20;
    rightPow = 20;
    Walker *walker10 = new Walker(leftPow, rightPow);
    Predicate *predicate10 = new WheelDistancePredicate(15, robotAPI);
    commandExecutor->addCommand(walker10, predicate10, GET_VARIABLE_NAME(walker10));
    commandExecutor->addCommand(colorPWMTracer, predicate10, GET_VARIABLE_NAME(colorPWMTracer));
    commandExecutor->addCommand(colorPWMTracer, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(colorPWMTracer));
    //青の後ちょっと進む
    //  Predicate *predicate10bl = new WheelDistancePredicate(10, robotAPI);
    //   commandExecutor->addCommand(walker10, predicate10bl, GET_VARIABLE_NAME(walker10));
    Stopper *stopper10 = new Stopper();
    Predicate *predicateS10 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper10, predicateS10, GET_VARIABLE_NAME(stoppper10));
    // １１ 指定角度右回転（青ラインに向く）
    CommandAndPredicate *predicate11 = new RotateRobotUseGyroCommandAndPredicate(4, 5, robotAPI);
    commandExecutor->addCommand(predicate11->getCommand(), predicate11->getPredicate(), GET_VARIABLE_NAME(predicate11->getCommand()));
    Stopper *stopper11 = new Stopper();
    Predicate *predicateS11 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    // 12「青検知するまで」直進
    leftPow = 20;
    rightPow = 20;
    Walker *walker12 = new Walker(leftPow, rightPow);
    Predicate *predicate12 = new WheelDistancePredicate(27, robotAPI);
    commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
    Walker *walker12b = new Walker(leftPow, rightPow);
    commandExecutor->addCommand(walker12b, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(walker12b));
    commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    bool useFacingAngle0 = false;
    if (useFacingAngle0) // FacingAngleによるガレージイン
    {
      float straightKp = 1;
      float straightKi = 0;
      float straightKd = 1;
      float straightDt = 1;
      float pwm = 20;
      float distance = 8;
      PIDStraightWalker *straightWalker = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *straightWalkerPredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(straightWalker, straightWalkerPredicate, GET_VARIABLE_NAME(straightWalker));
      float faKp = 0.3;
      float faKi = 0.015;
      float faKd = 0.7;
      float faDt = 1;
      float waitFaUsec = 2000000;
      float angle = 0;
      PIDFacingAngleAbs *facing0 = new PIDFacingAngleAbs(FA_WheelCount, angle, faKp, faKi, faKd, faDt);
      commandExecutor->addCommand(facing0, new ORPredicate(new TimerPredicate(waitFaUsec), new FinishedCommandPredicate(facing0)), GET_VARIABLE_NAME(facing0));
    }
    else // 青トレースによるガレージイン
    {
      // 13,10ど右回転 ８回
      bool gyro = false;
      if (gyro == true)
      { //ジャイロで調節するならtrueに
        CommandAndPredicate *predicate13 = new RotateRobotUseGyroCommandAndPredicate(40, 5, robotAPI);
        Walker *walker13S = new Walker(leftPow, rightPow);
        Predicate *predicate13S = new WheelDistancePredicate(1, robotAPI);
        for (int i = 0; i < 2; i++)
        {
          commandExecutor->addCommand(predicate13->getCommand(), predicate13->getPredicate(), GET_VARIABLE_NAME(predicate13->getCommand()));
          commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
          commandExecutor->addCommand(walker13S, predicate13S, GET_VARIABLE_NAME(walker13S));
          commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
        }
      }
      else
      {
        leftPow = 10;
        rightPow = -10;
        Walker *walker2 = new Walker(leftPow, rightPow);
        Predicate *predicate2 = new WheelDistancePredicate(10, robotAPI);
        commandExecutor->addCommand(walker2, predicate2, GET_VARIABLE_NAME(walker2));
      }
      ColorPIDTracerV2 *bluePWMTracer = new ColorPIDTracerV2(LEFT_TRACE, Trace_R, 5, kp, ki, kd, dt, pidR);
      calibrator->addColorPIDTracer(bluePWMTracer);
      Predicate *predicate13blue = new WheelDistancePredicate(10, robotAPI);
      commandExecutor->addCommand(bluePWMTracer, predicate13blue, GET_VARIABLE_NAME(bluePWMTracer));
      commandExecutor->addCommand(stopper11, predicateS11, GET_VARIABLE_NAME(stoppper11));
    }
    // 14,ガレージに直進（取得した色ごとに分岐させる）colorReadergetColor()で色を取得できる
    Command *dealingWithGarage14 = new DealingWithGarage(garageCardColorPtr, commandExecutor, false);
    Predicate *predicate14 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(dealingWithGarage14, predicate14, GET_VARIABLE_NAME(dealingWithGarage14));
  }
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
// ColorIDReaderModeの場合のcommandExecutor初期化処理
#ifdef ColorIDReaderMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // rgbRawReaderの初期化とCommandExecutorへの追加
  ColorIDReader *reader = new ColorIDReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(reader, startButtonPredicate, GET_VARIABLE_NAME(reader));
}
#endif
#ifdef BrightnessReaderMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  BrightnessReader *reader = new BrightnessReader();
  commandExecutor->addCommand(reader, new Predicate(), GET_VARIABLE_NAME(reader));
}
#endif
// Rotate360TestModeの場合のcommandExecutor初期化処理
#if defined(Rotate360TestMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  float motorRotateAngle = 540; // ここの値をいじってはかって
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  float pwm = 10;
  Walker *walker = new Walker(pwm, -pwm); // 右に向く
  Predicate *walkerPredicate = new MotorCountPredicate(robotAPI->getLeftWheel(), motorRotateAngle);
  commandExecutor->addCommand(walker, walkerPredicate, GET_VARIABLE_NAME(walker));
  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
}
#endif
// RotateTestModeの場合のcommandExecutor初期化処理
#ifdef RotateTestMode
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, ""); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  float angle = 10;
  float pwm = 15;
  RotateRobotCommandAndPredicate *commandAndPredicate = new RotateRobotCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), "rotateRobot");
  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif

void main_task(intptr_t unused)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    const uint32_t sleepDuration = 100 * 1000;
    const int8_t line_height = 20;

// Bluetoothファイルを開く
#ifdef EnableBluetooth
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
#endif

    // 初期化中メッセージの出力
    int line = 1;
    ev3_lcd_fill_rect(0, line * line_height, EV3_LCD_WIDTH, line_height, EV3_LCD_WHITE);
    ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示
    ev3_lcd_draw_string("initializing...", 0, line * line_height);

    // すべての周期ハンドラを止める
    // stp_cyc_all();

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
    robotAPI->getClock()->sleep(sleepDuration);
    robotAPI->reset();
    robotAPI->getClock()->sleep(sleepDuration);
    delete stopper;
    vector<string> resetedMessageLines;
    resetedMessageLines.push_back("reseted api");
    PrintMessage *printResetedMessage = new PrintMessage(resetedMessageLines, true);
    printResetedMessage->run(robotAPI);
    delete printResetedMessage;
    robotAPI->getClock()->sleep(sleepDuration);

    // バッテリーが少なかったら音で通知する
    if (lowBatteryVoltageMv >= ev3_battery_voltage_mV())
    {
      StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
      startDededon->run(robotAPI);
      delete startDededon;
    }

    // commandExecutorを初期化する（挙動定義）
    initializeCommandExecutor(commandExecutor, robotAPI);
#ifdef Right
    commandExecutor->reverseCommandAndPredicate();
#endif

    // デデドン！
    dededonCommandExecutor = new CommandExecutor(robotAPI, false);
    initDededon();

// FroggySongを歌うCommandExecutorを初期化する
#ifdef SingASong
    singASongCommandExecutor = new CommandExecutor(robotAPI, false);
    initSong(loopSong);
    commandExecutor->addCommand(new StartCyc(SING_A_SONG_CYC), new NumberOfTimesPredicate(1), "sing a song");
#endif

    vector<string> readyMessageLines;
    readyMessageLines.push_back("ready");
    PrintMessage *printReadyMessage = new PrintMessage(resetedMessageLines, true);
    printReadyMessage->run(robotAPI);
    delete printReadyMessage;

    // commandExecutor->run()の周期ハンドラを起動する
    StartCyc *startRunnerCyc = new StartCyc(RUNNER_CYC);
    startRunnerCyc->run(robotAPI);
    delete startRunnerCyc;

#ifdef EnableBluetooth
    // bluetoothCommandを受け取る周期ハンドラを起動する
    StartCyc *startListenBluetoothCyc = new StartCyc(LISTEN_BLUETOOTH_COMMAND_CYC);
    startListenBluetoothCyc->run(robotAPI);
    delete startListenBluetoothCyc;
#endif

    // 終了判定処理
    for (; true; clock->sleep(sleepDuration))
    {
      // 左ボタンが押されたら緊急停止のためにループを抜ける
      if (ev3_button_is_pressed(LEFT_BUTTON))
      {
        // 停止処理
        commandExecutor->emergencyStop();
        dededonCommandExecutor->emergencyStop();
#ifdef SingASong
        singASongCommandExecutor->emergencyStop();
#endif
        break;
      }

      // RUNNER_CYCが終了していたら走行完了なのでループを抜ける
      // CommandExecutorが終了していたら走行完了なのでループを抜ける
      if (commandExecutor->isFinished())
      {
        break;
      }
    }

    // runner_cycを止める
    stp_cyc(RUNNER_CYC);

#ifdef EnableBluetooth
    // LISTEN_BLUETOOTH_COMMAND_CYCが走っていたら止める。
    // T_RCYC btcCycState;
    // ref_cyc(LISTEN_BLUETOOTH_COMMAND_CYC, &btcCycState);
    // if (btcCycState.cycstat == TCYC_STA)
    // {
    stp_cyc(LISTEN_BLUETOOTH_COMMAND_CYC);
    // }
#endif

#ifdef SingASong
    // 歌ってたら止める
    // T_RCYC singASongCycState;
    // ref_cyc(RETURN_TO_START_POINT_CYC, &singASongCycState);
    // if (singASongCycState.cycstat == TCYC_STA)
    // {
    stp_cyc(SING_A_SONG_CYC);
    // }
#endif

    // returnToStartPointの終了待機
    robotAPI->getClock()->sleep(sleepDuration);
    for (; true; clock->sleep(sleepDuration))
    {
      // 左ボタンが押されたら周期ハンドラを止める
      if (ev3_button_is_pressed(LEFT_BUTTON))
      {
        // 停止処理
        stp_cyc(BTC_RETURN_TO_START_POINT);
      }

      // RETURN_TO_START_POINT_CYCが終了していたら走行完了なのでループを抜ける
      T_RCYC returnToStartPointCycState;
      ref_cyc(BTC_RETURN_TO_START_POINT, &returnToStartPointCycState);
      if (returnToStartPointCycState.cycstat == TCYC_STP)
      {
        break;
      }
    }

    stp_cyc_all();

    // 走行体停止
    stopper->run(robotAPI);
    delete stopper;

    // 終了メッセージの表示
    robotAPI->getClock()->sleep(sleepDuration);
    vector<string> messageLines;
    messageLines.push_back("finish!!");
    PrintMessage printFinishMessage(messageLines, true);
    printFinishMessage.run(robotAPI);

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
    delete facing90;
    delete facing180;
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }

#endif
#ifdef EnableBluetooth
  fclose(bt);
#endif
};
