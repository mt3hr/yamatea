#include "Setting.h"
#ifdef TrueCourceKomichiModeRegional

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

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;

  float leftPow;
  float rightPow;

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);

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

  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

// #define SonarStarter
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
  // 第一直進
  KleftPow = 50;
  KrightPow = 50;
  KwalkerS = new Walker(KleftPow, KrightPow);
  // Predicate *predicate1 = new WheelDistancePredicate(40, robotAPI);
  Predicate *Kpredicate1 = new WheelDistancePredicate(20, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate1, GET_VARIABLE_NAME(KwalkerS));
  // 第二カーブ
  KleftPow = 50;
  KrightPow = 10;
  Walker *Kwalker2 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate2 = new WheelDistancePredicate(24, robotAPI);
  commandExecutor->addCommand(Kwalker2, Kpredicate2, GET_VARIABLE_NAME(Kwalker2));
  // 第三直進
  KleftPow = 50;
  KrightPow = 50;
  // Walker *Kwalker3 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate3 = new WheelDistancePredicate(52, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate3, GET_VARIABLE_NAME(KwalkerS));
  // 第四カーブ
  KleftPow = 50;
  KrightPow = 10;
  Walker *Kwalker4 = new Walker(KleftPow, KrightPow);
  Predicate *Kpredicate4 = new WheelDistancePredicate(25, robotAPI);
  commandExecutor->addCommand(Kwalker4, Kpredicate4, GET_VARIABLE_NAME(Kwalker4));
  // 5直進
  Predicate *Kpredicate5 = new WheelDistancePredicate(10, robotAPI);
  commandExecutor->addCommand(KwalkerS, Kpredicate5, GET_VARIABLE_NAME(KwalkerS));
  // 第6カーブ,mid1
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
  // 第8　2度目交差点から抜ける
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
  // Walker *Kwalker10 = new Walker(KleftPow, KrightPow);
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