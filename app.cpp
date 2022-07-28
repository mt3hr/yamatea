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

bool enableCalibrateTargetBrightness = true; // PIDTracer.targetBrightnessをキャリブレーションするときはtrueにして
int targetBrightness = 20;                   // enableCalibrateTargetBrightnessがfalseのときに使われるtargetBrightnessの値

// 距離によるシーン切り替え用変数。MotorCountPredicate
int sceneBananaMotorCountPredicateArg = 1200;       // 8の字急カーブ突入前
int sceneOrangeMotorCountPredicateArg = 2450;       // 8の字クロス1回目突入前
int sceneStarFruitsMotorCountPredicateArg = 2550;   // 8の字クロス1回目通過後
int sceneCherryMotorCountPredicateArg = 2700;       // 8の字クロス1回目通過後ライントレース復帰時
int sceneWaterMelonMotorCountPredicateArg = 5990;   // 8の字クロス2回目突入前
int sceneBokChoyMotorCountPredicateArg = 6800;      // 8の時クロス2回目通過後直進中
int sceneDorianMotorCountPredicateArg = 7000;       // 8の字クロス2回目通過後ライントレース復帰時
int sceneMelonMotorCountPredicateArg = 9000;        // 中央直進突入後
int sceneCucumberMotorCountPredicateArg = 10800;    // 中央直進脱出前
int sceneStrawberryMotorCountPredicateArg = 100000; // ゴールまで

// EV3APIオブジェクトの初期化
TouchSensor touchSensor(PORT_1);
ColorSensor colorSensor(PORT_2);
Motor leftWheel(PORT_C);
Motor rightWheel(PORT_B);
Clock clock;
CommandExecutor *commandExecutor;

void initialize()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor();

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
  Predicate *predicateBanana = new MotorCountPredicate(&leftWheel, sceneBananaMotorCountPredicateArg);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 15;
  kp = 0.75;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicateOrange = new MotorCountPredicate(&leftWheel, sceneOrangeMotorCountPredicateArg);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange);

  // StarFruitsScenarioTracerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 20;
  ScenarioTracer *starFruitsScenarioTracer = new ScenarioTracer(leftPow, rightPow, &leftWheel, &rightWheel);
  Predicate *predicateStarFruits = new MotorCountPredicate(&leftWheel, sceneStarFruitsMotorCountPredicateArg);
  commandExecutor->addCommand(starFruitsScenarioTracer, predicateStarFruits);

  // CherryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *cherryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicateCherry = new MotorCountPredicate(&leftWheel, sceneCherryMotorCountPredicateArg);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry);

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicateWaterMelon = new MotorCountPredicate(&leftWheel, sceneWaterMelonMotorCountPredicateArg);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon);

  // BokChoyScenarioTracerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 20;
  ScenarioTracer *bokChoyScenarioTracer = new ScenarioTracer(leftPow, rightPow, &leftWheel, &rightWheel);
  Predicate *predicateBokChoy = new MotorCountPredicate(&leftWheel, sceneBokChoyMotorCountPredicateArg);
  commandExecutor->addCommand(bokChoyScenarioTracer, predicateBokChoy);

  // DorianPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *dorianPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicateDorian = new MotorCountPredicate(&leftWheel, sceneDorianMotorCountPredicateArg);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian);

  // MelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *melonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicateMelon = new MotorCountPredicate(&leftWheel, sceneMelonMotorCountPredicateArg);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon);

  // CucumberPIDTracerの初期化とCommandExecutorへの追加
  pwm = 30;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicateCucumber = new MotorCountPredicate(&leftWheel, sceneCucumberMotorCountPredicateArg);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber);

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.65;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicateStrawberry = new MotorCountPredicate(&leftWheel, sceneStrawberryMotorCountPredicateArg);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry);

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
  ext_tsk();
}