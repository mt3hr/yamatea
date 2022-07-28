// TOO メモリ管理してなくない？
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "Clock.h"

#include "app.h"
#include "util.h"

#include "Command.h"
#include "CommandExecutor.h"
#include "PIDTargetBrightnessCalibrator.h"
#include "Predicate.h"
#include "PIDTracer.h"
#include "ScenarioTracer.h"
#include "StartButtonPredicate.h"
#include "MotorCountPredicate.h"
#include "Handler.h"
#include "SetPIDTargetBrightnessWhenCalibratedHandler.h"

using namespace ev3api;

bool isRightCourse = false;                  // 左コースならfalse, 右コースならtrue。
bool enableCalibrateTargetBrightness = true; // PIDTracer.targetBrightnessをキャリブレーションするときはtrueにして
int targetBrightness = 20;                   // enableCalibrateTargetBrightnessがfalseのときに使われるtargetBrightnessの値

// EV3APIオブジェクトの初期化
TouchSensor touchSensor(PORT_1);
ColorSensor colorSensor(PORT_2);
Motor leftWheel(PORT_C);
Motor rightWheel(PORT_B);
Clock clock;
CommandExecutor *commandExecutor;

// PIDTracer反転関数。
// 左コースならそれをそのまま、右コースならば反転させたPIDTracerを返す
PIDTracer *ifRightThenReverseCommand(PIDTracer *pidTracer, bool isRightCource)
{
  if (isRightCource)
  {
    PIDTracer *reversed = pidTracer->generateReverseCommand();
    delete (pidTracer); // TODO いいのか？
    return reversed;
  }
  else
  {
    return pidTracer;
  }
}

// ScenarioTracer反転関数。
// 左コースならそれをそのまま、右コースならば反転させたScenarioTracerを返す
ScenarioTracer *ifRightThenReverseCommand(ScenarioTracer *scenarioTracer, bool isRightCource)
{
  if (isRightCource)
  {
    ScenarioTracer *reversed = scenarioTracer->generateReverseCommand();
    delete (scenarioTracer); // TODO いいのか？
    return reversed;
  }
  else
  {
    return scenarioTracer;
  }
}

// Predicate生成関数。
// isRightCourseがtrueならば右コース用の左車輪回転数Predicateを、
// falseならば左コース用の右車輪回転数Predicateを生成する。
MotorCountPredicate *generateMotorCountPredicate(bool isRightCource, int count)
{
  if (isRightCource)
  {
    return new MotorCountPredicate(&leftWheel, count);
  }
  else
  {
    return new MotorCountPredicate(&rightWheel, count);
  }
}

void initialize()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor();

  // 距離によるシーン切り替え用変数。MotorCountPredicate
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  int sceneBananaMotorCountPredicateArg = 1200;       // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int sceneOrangeMotorCountPredicateArg = 2450;       // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2550;   // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2700;       // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 5990;   // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  int sceneBokChoyMotorCountPredicateArg = 6800;      // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 7000;       // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneMelonMotorCountPredicateArg = 9000;        // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneCucumberMotorCountPredicateArg = 10800;    // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 100000; // ゴールまで。いちご好き。ライントレースする。

  // Commandの定義とCommandExecutorへの追加ここから

  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetBrightnessCalibrator *pidTargetBrightnessCalibrator = new PIDTargetBrightnessCalibrator(&colorSensor, &clock);
  Predicate *startButtonPredicate = new StartButtonPredicate(&touchSensor);
  if (enableCalibrateTargetBrightness)
  {
    commandExecutor->addCommand(pidTargetBrightnessCalibrator, startButtonPredicate);
  }
  else
  {
    // targetBrightnessをキャリブレーションしない場合
    commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
  }

  int pwm;
  float kp;
  float ki;
  float kd;
  int dt;

  int leftPow;
  int rightPow;

  // BananaPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  bananaPIDTracer = ifRightThenReverseCommand(bananaPIDTracer, isRightCourse);
  MotorCountPredicate *predicateBanana = generateMotorCountPredicate(isRightCourse, sceneBananaMotorCountPredicateArg);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 15;
  kp = 0.75;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  orangePIDTracer = ifRightThenReverseCommand(orangePIDTracer, isRightCourse);
  MotorCountPredicate *predicateOrange = generateMotorCountPredicate(isRightCourse, sceneOrangeMotorCountPredicateArg);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange);

  // StarFruitsScenarioTracerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 20;
  ScenarioTracer *starFruitsScenarioTracer = new ScenarioTracer(leftPow, rightPow, &leftWheel, &rightWheel);
  starFruitsScenarioTracer = ifRightThenReverseCommand(starFruitsScenarioTracer, isRightCourse);
  MotorCountPredicate *predicateStarFruits = generateMotorCountPredicate(isRightCourse, sceneStarFruitsMotorCountPredicateArg);
  commandExecutor->addCommand(starFruitsScenarioTracer, predicateStarFruits);

  // CherryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *cherryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  cherryPIDTracer = ifRightThenReverseCommand(cherryPIDTracer, isRightCourse);
  MotorCountPredicate *predicateCherry = generateMotorCountPredicate(isRightCourse, sceneCherryMotorCountPredicateArg);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry);

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  waterMelonPIDTracer = ifRightThenReverseCommand(waterMelonPIDTracer, isRightCourse);
  MotorCountPredicate *predicateWaterMelon = generateMotorCountPredicate(isRightCourse, sceneWaterMelonMotorCountPredicateArg);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon);

  // BokChoyScenarioTracerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 20;
  ScenarioTracer *bokChoyScenarioTracer = new ScenarioTracer(leftPow, rightPow, &leftWheel, &rightWheel);
  bokChoyScenarioTracer = ifRightThenReverseCommand(bokChoyScenarioTracer, isRightCourse);
  MotorCountPredicate *predicateBokChoy = generateMotorCountPredicate(isRightCourse, sceneBokChoyMotorCountPredicateArg);
  commandExecutor->addCommand(bokChoyScenarioTracer, predicateBokChoy);

  // DorianPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *dorianPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  dorianPIDTracer = ifRightThenReverseCommand(dorianPIDTracer, isRightCourse);
  MotorCountPredicate *predicateDorian = generateMotorCountPredicate(isRightCourse, sceneDorianMotorCountPredicateArg);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian);

  // MelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *melonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  melonPIDTracer = ifRightThenReverseCommand(melonPIDTracer, isRightCourse);
  MotorCountPredicate *predicateMelon = generateMotorCountPredicate(isRightCourse, sceneMelonMotorCountPredicateArg);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon);

  // CucumberPIDTracerの初期化とCommandExecutorへの追加
  pwm = 30;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  cucumberPIDTracer = ifRightThenReverseCommand(cucumberPIDTracer, isRightCourse);
  MotorCountPredicate *predicateCucumber = generateMotorCountPredicate(isRightCourse, sceneCucumberMotorCountPredicateArg);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber);

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.65;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  strawberryPIDTracer = ifRightThenReverseCommand(strawberryPIDTracer, isRightCourse);
  MotorCountPredicate *predicateStrawberry = generateMotorCountPredicate(isRightCourse, sceneStrawberryMotorCountPredicateArg);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry);

  // Commandの定義とCommandExecutorへの追加ここまで

  // キャリブレーションしたものをpidトレーサに反映するための処理。処理というかハンドラ追加。準備。
  if (enableCalibrateTargetBrightness)
  {
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(bananaPIDTracer, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(orangePIDTracer, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(cherryPIDTracer, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(waterMelonPIDTracer, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(dorianPIDTracer, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(melonPIDTracer, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(cucumberPIDTracer, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(strawberryPIDTracer, pidTargetBrightnessCalibrator));
  }
}

void tracer_task(intptr_t exinf)
{
  init_f("yamatea inited");
  commandExecutor->run();
  ext_tsk();
}

void main_task(intptr_t unused)
{
  const uint32_t duration = 100 * 1000;

  initialize();

  sta_cyc(TRACER_CYC);

  while (!ev3_button_is_pressed(LEFT_BUTTON))
  {
    clock.sleep(duration);
  }

  stp_cyc(TRACER_CYC);

  delete (commandExecutor); // TODO もっとdeleteしなきゃ

  ext_tsk();
}