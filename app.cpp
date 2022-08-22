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

#include "PrintMessage.h"
#include "Command.h"
#include "CommandExecutor.h"
#include "PIDTargetBrightnessCalibrator.h"
#include "Predicate.h"
#include "PIDTracer.h"
#include "Walker.h"
#include "DistanceReader.h"
#include "StartButtonPredicate.h"
#include "MotorCountPredicate.h"
#include "CommandAndPredicate.h"
#include "MotorRotateAnglePredicate.h"
#include "NumberOfTimesPredicate.h"
#include "SuitableForRightCourse.h"
#include "Stopper.h"
#include "RGBRawReader.h"
#include "DistancePredicate.h"
#include "RotateRobotCommandAndPredicate.h"
#include "FinishedCommandPredicate.h"
#include "CurvatureWalkerCommandAndPredicate.h"
#include "SwingSonarObstacleDetector.h"
#include "UFORunner.h"
#include "RobotAPI.h"
#include "GyroRotateAnglePredicate.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "DebugUtil.h"
#include "Bluetooth.h"

using namespace std;
using namespace ev3api;

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

// CommandExecutorとRobotAPIの宣言
CommandExecutor *commandExecutor;
RobotAPI *robotAPI;

// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#if defined(LeftCourceMode) | defined(RightCourceMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
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

  // Commandの定義とCommandExecutorへの追加ここから

  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetBrightnessCalibrator *pidTargetBrightnessCalibrator = new PIDTargetBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(pidTargetBrightnessCalibrator, startButtonPredicate);

  int pwm;
  float kp;
  float ki;
  float kd;
  int dt;

  int leftPow;
  int rightPow;

  // スタート後メッセージ出力コマンドの初期化とCommandExecutorへの追加
  vector<string> messageLines;
  messageLines.push_back("Started!!");
  messageLines.push_back("GOGOGO!!");
  PrintMessage *printMessage = new PrintMessage(messageLines, true);
  Predicate *printMessagePredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(printMessage, printMessagePredicate);

  // BananaPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  bananaPIDTracer = ifRightThenReverseCommand(bananaPIDTracer);
  MotorCountPredicate *predicateBanana = generateWheelCountPredicate(sceneBananaMotorCountPredicateArg);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana);
  pidTargetBrightnessCalibrator->addPIDTracer(bananaPIDTracer);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 15;
  kp = 0.75;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  orangePIDTracer = ifRightThenReverseCommand(orangePIDTracer);
  MotorCountPredicate *predicateOrange = generateWheelCountPredicate(sceneOrangeMotorCountPredicateArg);
  pidTargetBrightnessCalibrator->addPIDTracer(orangePIDTracer);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange);

  // StarFruitsWalkerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 20;
  Walker *starFruitsWalker = new Walker(leftPow, rightPow);
  starFruitsWalker = ifRightThenReverseCommand(starFruitsWalker);
  MotorCountPredicate *predicateStarFruits = generateWheelCountPredicate(sceneStarFruitsMotorCountPredicateArg);
  commandExecutor->addCommand(starFruitsWalker, predicateStarFruits);

  // CherryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *cherryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  cherryPIDTracer = ifRightThenReverseCommand(cherryPIDTracer);
  MotorCountPredicate *predicateCherry = generateWheelCountPredicate(sceneCherryMotorCountPredicateArg);
  pidTargetBrightnessCalibrator->addPIDTracer(cherryPIDTracer);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry);

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  waterMelonPIDTracer = ifRightThenReverseCommand(waterMelonPIDTracer);
  MotorCountPredicate *predicateWaterMelon = generateWheelCountPredicate(sceneWaterMelonMotorCountPredicateArg);
  pidTargetBrightnessCalibrator->addPIDTracer(waterMelonPIDTracer);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon);

  // BokChoyWalkerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 18;
  Walker *bokChoyWalker = new Walker(leftPow, rightPow);
  bokChoyWalker = ifRightThenReverseCommand(bokChoyWalker);
  MotorCountPredicate *predicateBokChoy = generateWheelCountPredicate(sceneBokChoyMotorCountPredicateArg);
  commandExecutor->addCommand(bokChoyWalker, predicateBokChoy);

  // DorianPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *dorianPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  dorianPIDTracer = ifRightThenReverseCommand(dorianPIDTracer);
  MotorCountPredicate *predicateDorian = generateWheelCountPredicate(sceneDorianMotorCountPredicateArg);
  pidTargetBrightnessCalibrator->addPIDTracer(dorianPIDTracer);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian);

  // MelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *melonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  melonPIDTracer = ifRightThenReverseCommand(melonPIDTracer);
  MotorCountPredicate *predicateMelon = generateWheelCountPredicate(sceneMelonMotorCountPredicateArg);
  pidTargetBrightnessCalibrator->addPIDTracer(melonPIDTracer);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon);

  // CucumberPIDTracerの初期化とCommandExecutorへの追加
  pwm = 28;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  cucumberPIDTracer = ifRightThenReverseCommand(cucumberPIDTracer);
  MotorCountPredicate *predicateCucumber = generateWheelCountPredicate(sceneCucumberMotorCountPredicateArg);
  pidTargetBrightnessCalibrator->addPIDTracer(cucumberPIDTracer);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber);

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
  strawberryPIDTracer = ifRightThenReverseCommand(strawberryPIDTracer);
  MotorCountPredicate *predicateStrawberry = generateWheelCountPredicate(sceneStrawberryMotorCountPredicateArg);
  pidTargetBrightnessCalibrator->addPIDTracer(strawberryPIDTracer);
  commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry);

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
}
#endif

// DistanceReaderModeの場合のcommandExecutor初期化処理
#ifdef DistanceReaderMode
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // distanceReaderの初期化とCommandExecutorへの追加
  DistanceReader *distanceReader = new DistanceReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(distanceReader, startButtonPredicate);
}
#endif

// RGBRawReaderModeの場合のcommandExecutor初期化処理
#ifdef RGBRawReaderMode
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // rgbRawReaderの初期化とCommandExecutorへの追加
  RGBRawReader *rgbRawReader = new RGBRawReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(rgbRawReader, startButtonPredicate);
}
#endif

// Rotate360TestModeの場合のcommandExecutor初期化処理
#if defined(Rotate360TestMode)
void initializeCommandExecutor()
{
  int motorRotateAngle = 540; // ここの値をいじってはかって

  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  Walker *walker = new Walker(pwm, -pwm); // 右に向く
  Predicate *walkerPredicate = new MotorCountPredicate(leftWheel, motorRotateAngle);
  commandExecutor->addCommand(walker, walkerPredicate);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
}
#endif

// RotateTestModeの場合のcommandExecutor初期化処理
#if defined(RotateTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int angle = 10;
  int pwm = 15;
  RotateRobotCommandAndPredicate *commandAndPredicate = new RotateRobotCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate());

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
}
#endif

// NOTE ジャイロ、 実機とシミュレータで左右判定が逆になる？
#if defined(RotateGyroTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  int angle = -30;
  RotateRobotUseGyroCommandAndPredicate *rotateRobotCommandAndPredicate = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
  commandExecutor->addCommand(rotateRobotCommandAndPredicate->getCommand(), rotateRobotCommandAndPredicate->getPredicate());

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
}
#endif

#ifdef StraightTestMode
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 直進コマンドの初期化とCommandExecutorへの追加
  int pwm = 50;
  float distanceCm = 50;
  Walker *walker = new Walker(pwm, pwm);
  DistancePredicate *walkerPredicate = new DistancePredicate(distanceCm, robotAPI);
  commandExecutor->addCommand(walker, walkerPredicate);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
}
#endif

#if defined(CurvatureWalkerTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 曲率進行コマンドの初期化とCommandExecutorへの追加
  int pwm = 20; // NOTE pwm上げるとおかしくなる
  float r = 20;
  float theta = 360;
  CurvatureWalkerCommandAndPredicate *commandAndPredicate = new CurvatureWalkerCommandAndPredicate(pwm, r, theta, robotAPI);
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate());

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
}
#endif

#if defined(SwingSonarDetectorTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 障害物検出コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  float swingLeft = 90.0;
  float swingRight = -90.0;
  int targetLeft = 20;
  int targetRight = 20;
  SwingSonarObstacleDetector *swingSonarDetector = new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT_CENTER, pwm, swingLeft, swingRight, targetLeft, targetRight);
  Predicate *swingSonarDetectorPredicate = new FinishedCommandPredicate(swingSonarDetector);
  commandExecutor->addCommand(swingSonarDetector, swingSonarDetectorPredicate);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
}
#endif

#if defined(ShigekiTestMode)
void initializeCommandExecutor()
{
  int pwm = 10;
  float acn = -30.91474484;
  float nc = 23.72114075;
  // float bcn = 1.022709978;
  float nTurn = 69.07498194;
  float n = 1.0;

  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // ACN度回転する
  RotateRobotCommandAndPredicate *turnACNCommandAndPredicate = new RotateRobotCommandAndPredicate(acn, pwm, robotAPI);
  commandExecutor->addCommand(turnACNCommandAndPredicate->getCommand(), turnACNCommandAndPredicate->getPredicate());

  // NCの距離進む
  Walker *walkNCCommand = new Walker(pwm, pwm);
  DistancePredicate *walkNCPredicate = new DistancePredicate(nc, robotAPI);
  commandExecutor->addCommand(walkNCCommand, walkNCPredicate);

  // nTurn分旋回する
  RotateRobotCommandAndPredicate *turnNCommandAndPredicate = new RotateRobotCommandAndPredicate(nTurn, pwm, robotAPI);
  commandExecutor->addCommand(turnNCommandAndPredicate->getCommand(), turnNCommandAndPredicate->getPredicate());

  // n分進む
  Walker *walkNCommand = new Walker(pwm, pwm);
  DistancePredicate *walkNPredicate = new DistancePredicate(n, robotAPI);
  commandExecutor->addCommand(walkNCommand, walkNPredicate);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
}
#endif

#if defined(UFORunnerTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // UFO走行コマンドの初期化とCommandExecutorへの追加
  float n = 7;
  int walkerPWM = 20;
  int rotatePWM = 10;
  float swingLeftAngle = 90.0;
  float swingRightAngle = 90.0;
  int targetLeftDistance = 20;
  int targetRightDistance = 20;
  bool reverseTest = true;

  UFORunner *ufoRunner = new UFORunner(n, walkerPWM, rotatePWM, swingLeftAngle, swingRightAngle, targetLeftDistance, targetRightDistance);
  if (reverseTest)
  {
    ufoRunner = ufoRunner->generateReverseCommand();
  }
  commandExecutor->addCommand(ufoRunner, new FinishedCommandPredicate(ufoRunner));

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate);
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
Walker *returnToStartPointTurnRightWalker = new Walker(20, -20);
Walker *returnToStartPointTurnLeftWalker = new Walker(20, -20);
colorid_t returnToStartPointEdgeLineColor = COLOR_RED;

void return_to_start_point_task(intptr_t exinf)
{
  switch (returnToStartPointState)
  {
  case RTSP_TURNNING_UP:
  {
    int targetAngle = 180;
    int angle = gyroSensor->getAngle();

    if (angle < targetAngle)
    {
      returnToStartPointTurnRightWalker->run(robotAPI);
    }
    else
    {
      returnToStartPointTurnLeftWalker->run(robotAPI);
    }

    // これのためだけにPredicate定義するのは嫌なので筋肉コーディングします
    if (angle > targetAngle - 2 && angle < targetAngle + 2)
    {
      returnToStartPointState = RTSP_WALKING_UP;
    }
  }
  break;

  case RTSP_WALKING_UP:
  {
    returnToStartPointStraightWalker->run(robotAPI);
    colorid_t colorID = colorSensor->getColorNumber();
    if (colorID == returnToStartPointEdgeLineColor)
    {
      returnToStartPointState = RTSP_TURNNING_RIGHT;
    }
  }
  break;

  case RTSP_TURNNING_RIGHT:
  {
    int targetAngle = 270;
    int angle = gyroSensor->getAngle();

    if (angle < targetAngle)
    {
      returnToStartPointTurnRightWalker->run(robotAPI);
    }
    else
    {
      returnToStartPointTurnLeftWalker->run(robotAPI);
    }

    // これのためだけにPredicate定義するのは嫌なので筋肉コーディングします
    if (angle > targetAngle - 2 && angle < targetAngle + 2)
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
  BTC_EMERGENCY_STOP = 1,
  BTC_RETURN_TO_START_POINT = 2,
};

void listen_bluetooth_command_task(intptr_t exinf)
{
#ifdef BluetoothMode
  const uint32_t sleepDuration = 1000 * 1000;

  char btCommand[20];
BTCLOOP:
  while (true)
  {
    fread(btCommand, sizeof(char), 20, bt);

    string btCommandStr = string(btCommand);
    switch (BTC_RETURN_TO_START_POINT)
    {

    case BTC_EMERGENCY_STOP:
    {
      commandExecutor->emergencyStop();
      break BTCLOOP;
    }
    case BTC_RETURN_TO_START_POINT:
    {
      commandExecutor->emergencyStop();

      // runnerTaskが終了するのを待機する
      while (true)
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
      break;
    }
    // ちょっと待つ
    clock->sleep(sleepDuration);
  }
#endif

  ext_tsk();
}

void main_task(intptr_t unused)
{
  ev3_lcd_set_font(EV3_FONT_MEDIUM);              // フォントの設定
  ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示

  const uint32_t sleepDuration = 100 * 1000;

  // robotAPIの初期化。完全停止してapiを初期化する
  robotAPI = new RobotAPI(touchSensor, colorSensor, sonarSensor, leftWheel, rightWheel, armMotor, tailMotor, gyroSensor, clock);
  Stopper *stopper = new Stopper();
  stopper->run(robotAPI);
  delete stopper;
  robotAPI->reset();
  writeDebug("reseted api");
  flushDebug(DEBUG, robotAPI);

  // commandExecutorを初期化する
  initializeCommandExecutor();

  writeDebug("ready");
  flushDebug(INFO, robotAPI);

  // commandExecutor->run()の周期ハンドラを起動する
  sta_cyc(RUNNER_CYC);

  // 終了判定処理
  while (true)
  {
    // 左ボタンが押されたら緊急停止のためにループを抜ける
    if (ev3_button_is_pressed(LEFT_BUTTON))
    {
      // 停止処理
      commandExecutor->emergencyStop();
      break;
    }

    // RUNNER_CYCが終了していたら走行完了なのでループを抜ける
    T_RCYC pk_rcyc;
    ref_cyc(RUNNER_CYC, &pk_rcyc);
    if (pk_rcyc.cycstat == TCYC_STP)
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

    // ちょっと待つ
    clock->sleep(sleepDuration);
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