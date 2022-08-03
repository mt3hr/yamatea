// TOOO メモリ管理してなくない？
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "SonarSensor.h"

#include "app.h"
#include "util.h"

#include "Command.h"
#include "CommandExecutor.h"
#include "WheelController.h"
#include "PIDTargetBrightnessCalibrator.h"
#include "Predicate.h"
#include "PIDTracer.h"
#include "ScenarioTracer.h"
#include "StartButtonPredicate.h"
#include "MotorCountPredicate.h"
#include "Handler.h"
#include "SetPIDTargetBrightnessWhenCalibratedHandler.h"
#include "CommandAndPredicate.h"
#include "MotorRotationAnglePredicate.h"

using namespace ev3api;

// ********** 設定ここから **********
// #define PrintMessage // コメントアウトを外すとコマンドの情報をディスプレイに表示する
bool isRightCourse = false;                  // 左コースならfalse, 右コースならtrue。
bool enableCalibrateTargetBrightness = true; // PIDTracer.targetBrightnessをキャリブレーションするときはtrueにして
int targetBrightness = 20;                   // enableCalibrateTargetBrightnessがfalseのときに使われるtargetBrightnessの値
// ********** 設定ここまで **********

// EV3APIオブジェクトの初期化
TouchSensor touchSensor(PORT_1);
ColorSensor colorSensor(PORT_2);
SonarSensor sonarSensor(PORT_3);
Motor leftWheel(PORT_C);
Motor rightWheel(PORT_B);
Clock clock;
CommandExecutor *commandExecutor;
WheelController *wheelController = new WheelController(&leftWheel, &rightWheel);

// ロボット旋回コマンド生成関数
CommandAndPredicate *generateRotationRobotCommand(int targetAngle)
{
  int angleFor360Turn = 10; // TODO 360度旋回するのに必要な左右車輪回転角度数

  int pwm = 10;
  int angle = ((float)targetAngle) / ((float)angleFor360Turn) * ((float)360);
  Command *command;
  if (targetAngle > 0)
  {
    command = new ScenarioTracer(pwm, -pwm, wheelController); // 右に向く
  }
  else
  {
    command = new ScenarioTracer(-pwm, pwm, wheelController); // 左に向く
  }

  Predicate *predicate = new MotorRotationAnglePredicate(angle, &leftWheel);

  return new CommandAndPredicate(command, predicate);
}

// PIDTracer反転関数。
// 左コースならそれをそのまま、右コースならば反転させたPIDTracerを返す
PIDTracer *ifRightThenReverseCommand(PIDTracer *pidTracer, bool isRightCource)
{
  if (isRightCource)
  {
    PIDTracer *reversed = pidTracer->generateReverseCommand();
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

void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(&leftWheel, &rightWheel);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // 距離によるシーン切り替え用変数。MotorCountPredicate
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  int sceneBananaMotorCountPredicateArg = 1200;       // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int sceneOrangeMotorCountPredicateArg = 2450;       // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2550;   // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2700;       // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 5150;   // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  int sceneBokChoyMotorCountPredicateArg = 5400;      // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 5700;       // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneMelonMotorCountPredicateArg = 8000;        // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneCucumberMotorCountPredicateArg = 9700;     // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 100000; // ゴールまで。いちご好き。ライントレースする。

  // Commandの定義とCommandExecutorへの追加ここから

  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetBrightnessCalibrator *pidTargetBrightnessCalibrator = new PIDTargetBrightnessCalibrator(&colorSensor, &clock);
  Predicate *startButtonPredicate = new StartButtonPredicate(&touchSensor);
  if (enableCalibrateTargetBrightness)
  {
    commandExecutor->addCommand(pidTargetBrightnessCalibrator, startButtonPredicate, doNothingHandler);
  }
  else
  {
    // targetBrightnessをキャリブレーションしない場合
    commandExecutor->addCommand(new Command(), startButtonPredicate, doNothingHandler); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
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
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  bananaPIDTracer = ifRightThenReverseCommand(bananaPIDTracer, isRightCourse);
  MotorCountPredicate *predicateBanana = generateMotorCountPredicate(isRightCourse, sceneBananaMotorCountPredicateArg);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana, doNothingHandler);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 15;
  kp = 0.75;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  orangePIDTracer = ifRightThenReverseCommand(orangePIDTracer, isRightCourse);
  MotorCountPredicate *predicateOrange = generateMotorCountPredicate(isRightCourse, sceneOrangeMotorCountPredicateArg);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, doNothingHandler);

  // StarFruitsScenarioTracerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 20;
  ScenarioTracer *starFruitsScenarioTracer = new ScenarioTracer(leftPow, rightPow, wheelController);
  starFruitsScenarioTracer = ifRightThenReverseCommand(starFruitsScenarioTracer, isRightCourse);
  MotorCountPredicate *predicateStarFruits = generateMotorCountPredicate(isRightCourse, sceneStarFruitsMotorCountPredicateArg);
  commandExecutor->addCommand(starFruitsScenarioTracer, predicateStarFruits, doNothingHandler);

  // CherryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *cherryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  cherryPIDTracer = ifRightThenReverseCommand(cherryPIDTracer, isRightCourse);
  MotorCountPredicate *predicateCherry = generateMotorCountPredicate(isRightCourse, sceneCherryMotorCountPredicateArg);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry, doNothingHandler);

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  waterMelonPIDTracer = ifRightThenReverseCommand(waterMelonPIDTracer, isRightCourse);
  MotorCountPredicate *predicateWaterMelon = generateMotorCountPredicate(isRightCourse, sceneWaterMelonMotorCountPredicateArg);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, doNothingHandler);

  // BokChoyScenarioTracerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 18;
  ScenarioTracer *bokChoyScenarioTracer = new ScenarioTracer(leftPow, rightPow, wheelController);
  bokChoyScenarioTracer = ifRightThenReverseCommand(bokChoyScenarioTracer, isRightCourse);
  MotorCountPredicate *predicateBokChoy = generateMotorCountPredicate(isRightCourse, sceneBokChoyMotorCountPredicateArg);
  commandExecutor->addCommand(bokChoyScenarioTracer, predicateBokChoy, doNothingHandler);

  // DorianPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *dorianPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  dorianPIDTracer = ifRightThenReverseCommand(dorianPIDTracer, isRightCourse);
  MotorCountPredicate *predicateDorian = generateMotorCountPredicate(isRightCourse, sceneDorianMotorCountPredicateArg);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian, doNothingHandler);

  // MelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *melonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  melonPIDTracer = ifRightThenReverseCommand(melonPIDTracer, isRightCourse);
  MotorCountPredicate *predicateMelon = generateMotorCountPredicate(isRightCourse, sceneMelonMotorCountPredicateArg);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon, doNothingHandler);

  // CucumberPIDTracerの初期化とCommandExecutorへの追加
  pwm = 28;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  cucumberPIDTracer = ifRightThenReverseCommand(cucumberPIDTracer, isRightCourse);
  MotorCountPredicate *predicateCucumber = generateMotorCountPredicate(isRightCourse, sceneCucumberMotorCountPredicateArg);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, doNothingHandler);

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, wheelController, &colorSensor);
  strawberryPIDTracer = ifRightThenReverseCommand(strawberryPIDTracer, isRightCourse);
  MotorCountPredicate *predicateStrawberry = generateMotorCountPredicate(isRightCourse, sceneStrawberryMotorCountPredicateArg);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, doNothingHandler);

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

  initializeCommandExecutor();

  sta_cyc(TRACER_CYC);

  while (!ev3_button_is_pressed(LEFT_BUTTON))
  {
    clock.sleep(duration);
  }

  stp_cyc(TRACER_CYC);
  commandExecutor->emergencyStop();

  ext_tsk();
}