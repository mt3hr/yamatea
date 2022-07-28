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
int scene1MotorCountPredicateArg = 2450;  // 8の字クロス1回目突入前
int scene2MotorCountPredicateArg = 2550;  // 8の字クロス1回目通過後
int scene3MotorCountPredicateArg = 5990;  // 8の字クロス2回目突入前
int scene4MotorCountPredicateArg = 6600;  // 8の時クロス2回目通過後直進中
int scene5MotorCountPredicateArg = 7000;  // 8の字クロス2回目通過後ライントレース復帰時
int scene6MotorCountPredicateArg = 9000;  // 中央直進突入後
int scene7MotorCountPredicateArg = 10800; // 中央直進脱出前

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

  // PIDTracer1の初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *commandPIDTracer1 = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicatePIDTracer1 = new MotorCountPredicate(&leftWheel, scene1MotorCountPredicateArg);
  commandExecutor->addCommand(commandPIDTracer1, predicatePIDTracer1);

  // ScenarioTracer1の初期化とCommandExecutorへの追加
  leftPow = 13;
  rightPow = 20;
  ScenarioTracer *commandScenarioTracer1 = new ScenarioTracer(leftPow, rightPow, &leftWheel, &rightWheel);
  Predicate *predicateScenarioTracer1 = new MotorCountPredicate(&leftWheel, scene2MotorCountPredicateArg);
  commandExecutor->addCommand(commandScenarioTracer1, predicateScenarioTracer1);

  // PIDTracer2の初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *commandPIDTracer2 = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicatePIDTracer2 = new MotorCountPredicate(&leftWheel, scene3MotorCountPredicateArg);
  commandExecutor->addCommand(commandPIDTracer2, predicatePIDTracer2);

  // ScenarioTracer2の初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 20;
  ScenarioTracer *commandScenarioTracer2 = new ScenarioTracer(leftPow, rightPow, &leftWheel, &rightWheel);
  Predicate *predicateScenarioTracer2 = new MotorCountPredicate(&leftWheel, scene4MotorCountPredicateArg);
  commandExecutor->addCommand(commandScenarioTracer2, predicateScenarioTracer2);

  // PIDTracer3の初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *commandPIDTracer3 = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicatePIDTracer3 = new MotorCountPredicate(&leftWheel, scene5MotorCountPredicateArg);
  commandExecutor->addCommand(commandPIDTracer3, predicatePIDTracer3);

  // PIDTracer4の初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *commandPIDTracer4 = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicatePIDTracer4 = new MotorCountPredicate(&leftWheel, scene6MotorCountPredicateArg);
  commandExecutor->addCommand(commandPIDTracer4, predicatePIDTracer4);

  // PIDTracer5の初期化とCommandExecutorへの追加
  pwm = 35;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *commandPIDTracer5 = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicatePIDTracer5 = new MotorCountPredicate(&leftWheel, scene7MotorCountPredicateArg);
  commandExecutor->addCommand(commandPIDTracer5, predicatePIDTracer5);

  // PIDTracer6の初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.8;
  ki = 0.2;
  kd = 0.8;
  dt = 1;
  PIDTracer *commandPIDTracer6 = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
  Predicate *predicatePIDTracer6 = new MotorCountPredicate(&leftWheel, 100000);
  commandExecutor->addCommand(commandPIDTracer6, predicatePIDTracer6);

  // キャリブレーションしたものをpidトレーサに反映するための処理
  if (enableCalibrateTargetBrightness)
  {
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(commandPIDTracer1, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(commandPIDTracer2, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(commandPIDTracer3, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(commandPIDTracer4, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(commandPIDTracer5, pidTargetBrightnessCalibrator));
    pidTargetBrightnessCalibrator->addRoadedHandler(new SetPIDTargetBrightnessWhenCalibratedHandler(commandPIDTracer6, pidTargetBrightnessCalibrator));
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