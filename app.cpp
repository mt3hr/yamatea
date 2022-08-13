#include "app.h"

#include "ev3api.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "SonarSensor.h"

#include "string"
#include "sstream"
#include "vector"

#include "Setting.h"
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
#include "Handler.h"
#include "SetPIDTargetBrightnessWhenCalibratedHandler.h"
#include "CommandAndPredicate.h"
#include "MotorRotateAnglePredicate.h"
#include "NumberOfTimesPredicate.h"
#include "SuitableForRightCourse.h"
#include "Stopper.h"
#include "RGBRawReader.h"
#include "DistancePredicate.h"
#include "ExecutePreparationWhenExitBeforeCommandHandler.h"
#include "RotateRobot.h"
#include "FinishedCommandPredicate.h"
#include "CurvatureWalker.h"
#include "SwingSonarObstacleDetector.h"
#include "UFORunner.h"
#include "RobotAPI.h"

using namespace std;
using namespace ev3api;

// 設定用準備ここから
bool enableCalibrateTargetBrightness = true;
int targetBrightness = 20;
// 設定用準備ここまで

// ********** 設定ここから **********

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
#define LeftCourceMode // 左コース用プログラム
//#define RightCourceMode // 右コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム
//#define RGBRawReaderMode    // RGBRawの値をはかるプログラム
//#define Rotate360TestMode // 360度回転に必要なモータ回転角をはかるためのもの。テスト用
//#define RotateTestMode // 旋回モード。テスト用
//#define StraightTestMode // 直進モード。テスト用
//#define CurvatureWalkerTestMode // 曲率旋回モード。テスト用
//#define SwingSonarDetectorTestMode // 障害物距離角度首振り検出モード。テスト用
//#define ShigekiTestMode // あなたの墓地にあり伝説でないカードＸ枚を対象とする。それらをあなたの手札に戻す。テスト用
//#define UFORunnerTestMode // UFO走行モード。テスト
// モード設定ここまで

void setting()
{
  wheelDiameter = 10.4;                  // 車輪直径。センチメートル。
  distanceFromSonarSensorToAxle = 10.5;  // ソナーセンサから車軸までの距離
  angleFor360TurnLeftRotateRobot = 520;  // 左に360度旋回するのに必要な左右車輪回転角度数
  angleFor360TurnRightRotateRobot = 510; // 右に360度旋回するのに必要な左右車輪回転角度数
  // TODO angleFor360の左右対応が逆になってるっぽいな

  wheelSpace = 14.5; // 左車輪と右車輪の間隔

  // LeftCourceMode, RightCourceModeの設定ここから
  enableCalibrateTargetBrightness = true; // PIDTracer.targetBrightnessをキャリブレーションするときはtrueにして
  targetBrightness = 20;                  // enableCalibrateTargetBrightnessがfalseのときに使われるtargetBrightnessの値
  // LeftCourceMode, RightCourceModeの設定ここまで

  // 情報出力の有効無効設定ここから
  enablePrintDebugMessage = true;        // trueにするとDebugUtil経由で出力されたメッセージを出力する
  enablePrintMessageMode = false;        // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
  enablePrintMessageForConsole = true;   // trueにすると、コンソールにも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）
  enablePrintMessageForBluetooth = true; // trueにすると、Bluetooth接続端末にも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）trueにする場合、すぐ下の行、#define EnableBluetoothのコメントアウトも外して。
#define EnableBluetooth                  // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ
  // 情報出力の有効無効設定ここまで
}

// ********** 設定ここまで **********

// 設定反映処理
bool isRightCourse =
#if defined(RightCourceMode)
    true;
#elif defined(LeftCourceMode)
    false;
#else
    false;
#endif

// サンプルのutil.cppから引っ張ってきたやつ
// char*ではなくstd::stringで受け取る
void init_f(string str)
{
  // フォントの設定と0行目の表示
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string(str.c_str(), 0, 0);
}

// EV3APIオブジェクトの初期化
TouchSensor *touchSensor = new TouchSensor(PORT_1);
ColorSensor *colorSensor = new ColorSensor(PORT_2);
SonarSensor *sonarSensor = new SonarSensor(PORT_3);
Motor *leftWheel = new Motor(PORT_C);
Motor *rightWheel = new Motor(PORT_B);
Motor *armMotor = new Motor(PORT_A);
Clock *clock = new Clock();

// CommandExecutorの宣言とRobotAPIの初期化
CommandExecutor *commandExecutor;
RobotAPI *robotAPI = new RobotAPI(touchSensor, colorSensor, sonarSensor, leftWheel, rightWheel, armMotor, clock);

// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#if defined(LeftCourceMode) | defined(RightCourceMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // 距離によるシーン切り替え用変数。MotorCountPredicateにわたす引数
  // そのシーンが終了する距離の定義。
  // シーン命名は野菜果物。（数字で管理するとシーン挿入時の修正が面倒くさいので）
  int sceneBananaMotorCountPredicateArg = 1200;      // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
  int sceneOrangeMotorCountPredicateArg = 2450;      // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
  int sceneStarFruitsMotorCountPredicateArg = 2550;  // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
  int sceneCherryMotorCountPredicateArg = 2700;      // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneWaterMelonMotorCountPredicateArg = 5150;  // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
  int sceneBokChoyMotorCountPredicateArg = 5400;     // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
  int sceneDorianMotorCountPredicateArg = 5700;      // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
  int sceneMelonMotorCountPredicateArg = 8000;       // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
  int sceneCucumberMotorCountPredicateArg = 9700;    // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
  int sceneStrawberryMotorCountPredicateArg = 10000; // ゴールまで。いちご好き。ライントレースする。

  // Commandの定義とCommandExecutorへの追加ここから

  // PIDTargetCalibratorの初期化とCommandExecutorへの追加
  PIDTargetBrightnessCalibrator *pidTargetBrightnessCalibrator = new PIDTargetBrightnessCalibrator(robotAPI);
  Predicate *startButtonPredicate = new StartButtonPredicate();
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

  // スタート後メッセージ出力コマンドの初期化とCommandExecutorへの追加
  vector<string> messageLines;
  messageLines.push_back("Started!!");
  messageLines.push_back("GOGOGO!!");
  PrintMessage *printMessage = new PrintMessage(messageLines, true);
  Predicate *printMessagePredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(printMessage, printMessagePredicate, doNothingHandler);

  // BananaPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  bananaPIDTracer = ifRightThenReverseCommand(bananaPIDTracer, isRightCourse);
  MotorCountPredicate *predicateBanana = generateMotorCountPredicate(isRightCourse, sceneBananaMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(bananaPIDTracer, predicateBanana, doNothingHandler);

  // OrangePIDTracerの初期化とCommandExecutorへの追加
  pwm = 15;
  kp = 0.75;
  ki = 0.2;
  kd = 0.65;
  dt = 1;
  PIDTracer *orangePIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  orangePIDTracer = ifRightThenReverseCommand(orangePIDTracer, isRightCourse);
  MotorCountPredicate *predicateOrange = generateMotorCountPredicate(isRightCourse, sceneOrangeMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(orangePIDTracer, predicateOrange, doNothingHandler);

  // StarFruitsWalkerの初期化とCommandExecutorへの追加
  leftPow = 16;
  rightPow = 20;
  Walker *starFruitsWalker = new Walker(leftPow, rightPow);
  starFruitsWalker = ifRightThenReverseCommand(starFruitsWalker, isRightCourse);
  MotorCountPredicate *predicateStarFruits = generateMotorCountPredicate(isRightCourse, sceneStarFruitsMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(starFruitsWalker, predicateStarFruits, doNothingHandler);

  // CherryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *cherryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  cherryPIDTracer = ifRightThenReverseCommand(cherryPIDTracer, isRightCourse);
  MotorCountPredicate *predicateCherry = generateMotorCountPredicate(isRightCourse, sceneCherryMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(cherryPIDTracer, predicateCherry, doNothingHandler);

  // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 18;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *waterMelonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  waterMelonPIDTracer = ifRightThenReverseCommand(waterMelonPIDTracer, isRightCourse);
  MotorCountPredicate *predicateWaterMelon = generateMotorCountPredicate(isRightCourse, sceneWaterMelonMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, doNothingHandler);

  // BokChoyWalkerの初期化とCommandExecutorへの追加
  leftPow = 20;
  rightPow = 18;
  Walker *bokChoyWalker = new Walker(leftPow, rightPow);
  bokChoyWalker = ifRightThenReverseCommand(bokChoyWalker, isRightCourse);
  MotorCountPredicate *predicateBokChoy = generateMotorCountPredicate(isRightCourse, sceneBokChoyMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(bokChoyWalker, predicateBokChoy, doNothingHandler);

  // DorianPIDTracerの初期化とCommandExecutorへの追加
  pwm = 10;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *dorianPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  dorianPIDTracer = ifRightThenReverseCommand(dorianPIDTracer, isRightCourse);
  MotorCountPredicate *predicateDorian = generateMotorCountPredicate(isRightCourse, sceneDorianMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(dorianPIDTracer, predicateDorian, doNothingHandler);

  // MelonPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.7;
  ki = 0.2;
  kd = 0.7;
  dt = 1;
  PIDTracer *melonPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  melonPIDTracer = ifRightThenReverseCommand(melonPIDTracer, isRightCourse);
  MotorCountPredicate *predicateMelon = generateMotorCountPredicate(isRightCourse, sceneMelonMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(melonPIDTracer, predicateMelon, doNothingHandler);

  // CucumberPIDTracerの初期化とCommandExecutorへの追加
  pwm = 28;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *cucumberPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  cucumberPIDTracer = ifRightThenReverseCommand(cucumberPIDTracer, isRightCourse);
  MotorCountPredicate *predicateCucumber = generateMotorCountPredicate(isRightCourse, sceneCucumberMotorCountPredicateArg, robotAPI);
  commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, doNothingHandler);

  // StrawberryPIDTracerの初期化とCommandExecutorへの追加
  pwm = 20;
  kp = 0.6;
  ki = 0.2;
  kd = 0.6;
  dt = 1;
  PIDTracer *strawberryPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness);
  strawberryPIDTracer = ifRightThenReverseCommand(strawberryPIDTracer, isRightCourse);
  MotorCountPredicate *predicateStrawberry = generateMotorCountPredicate(isRightCourse, sceneStrawberryMotorCountPredicateArg, robotAPI);
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
#endif

// DistanceReaderModeの場合のcommandExecutor初期化処理
#ifdef DistanceReaderMode
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // distanceReaderの初期化とCommandExecutorへの追加
  DistanceReader *distanceReader = new DistanceReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(distanceReader, startButtonPredicate, doNothingHandler);
}
#endif

// RGBRawReaderModeの場合のcommandExecutor初期化処理
#ifdef RGBRawReaderMode
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // rgbRawReaderの初期化とCommandExecutorへの追加
  RGBRawReader *rgbRawReader = new RGBRawReader();
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(rgbRawReader, startButtonPredicate, doNothingHandler);
}
#endif

// Rotate360TestModeの場合のcommandExecutor初期化処理
#if defined(Rotate360TestMode)
void initializeCommandExecutor()
{
  int motorRotateAngle = 540; // ここの値をいじってはかって

  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, doNothingHandler); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  Walker *walker = new Walker(pwm, -pwm); // 右に向く
  Predicate *walkerPredicate = new MotorCountPredicate(leftWheel, motorRotateAngle, false);
  commandExecutor->addCommand(walker, walkerPredicate, doNothingHandler);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, doNothingHandler);
}
#endif

// RotateTestModeの場合のcommandExecutor初期化処理
#if defined(RotateTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  // 走行体回転コマンドの初期化とCommandExecutorへの追加
  int angle = 10;
  int pwm = 15;
  Predicate *startButtonPredicate = new StartButtonPredicate();
  CommandAndPredicate *commandAndPredicate = generateRotateRobotCommand(angle, pwm, robotAPI);
  commandExecutor->addCommand(new Command(), startButtonPredicate, commandAndPredicate->getPreHandler()); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), doNothingHandler);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, doNothingHandler);
}
#endif

#ifdef StraightTestMode
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  // 直進コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();

  int pwm = 50;
  float distanceCm = 50;
  Walker *walker = new Walker(pwm, pwm);
  DistancePredicate *walkerPredicate = new DistancePredicate(distanceCm, robotAPI->getLeftWheel());

  Handler *startButtonExitHandler = new ExecutePreparationWhenExitBeforeCommandHandler(walkerPredicate);

  commandExecutor->addCommand(new Command(), startButtonPredicate, startButtonExitHandler); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
  commandExecutor->addCommand(walker, walkerPredicate, doNothingHandler);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, doNothingHandler);
}
#endif

#if defined(CurvatureWalkerTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  // 曲率進行コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();

  int pwm = 20; // TODO pwm上げるとおかしくなる
  float r = 20;
  float theta = 360;
  CommandAndPredicate *commandAndPredicate = generateCurvatureWalkerWithTheta(pwm, r, theta, false, robotAPI);

  commandExecutor->addCommand(new Command(), startButtonPredicate, commandAndPredicate->getPreHandler()); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
  commandExecutor->addCommand(commandAndPredicate->getCommand(), commandAndPredicate->getPredicate(), doNothingHandler);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, doNothingHandler);
}
#endif

#if defined(SwingSonarDetectorTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, doNothingHandler); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // 障害物検出コマンドの初期化とCommandExecutorへの追加
  int pwm = 10;
  float swingLeft = 90.0;
  float swingRight = -90.0;
  int targetLeft = 20;
  int targetRight = 20;
  SwingSonarObstacleDetector *swingSonarDetector = new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, pwm, swingLeft, swingRight, targetLeft, targetRight);
  Predicate *swingSonarDetectorPredicate = new FinishedCommandPredicate(swingSonarDetector);
  commandExecutor->addCommand(swingSonarDetector, swingSonarDetectorPredicate, doNothingHandler);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, doNothingHandler);
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

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  // ACN度回転する
  // NCの距離進む
  // nTurn分旋回する
  // n分進む
  Predicate *startButtonPredicate = new StartButtonPredicate();

  CommandAndPredicate *turnACNCommandAndPredicate = generateRotateRobotCommand(acn, pwm, robotAPI);

  Walker *walkNCCommand = new Walker(pwm, pwm);
  DistancePredicate *walkNCPredicate = new DistancePredicate(nc, leftWheel);

  CommandAndPredicate *turnNCommandAndPredicate = generateRotateRobotCommand(nTurn, pwm, robotAPI);

  Walker *walkNCommand = new Walker(pwm, pwm);
  DistancePredicate *walkNPredicate = new DistancePredicate(n, leftWheel);

  commandExecutor->addCommand(new Command(), startButtonPredicate, turnACNCommandAndPredicate->getPreHandler()); // なにもしないコマンドでタッチセンサがプレスされるのを待つ
  commandExecutor->addCommand(turnACNCommandAndPredicate->getCommand(), turnACNCommandAndPredicate->getPredicate(), new ExecutePreparationWhenExitBeforeCommandHandler(walkNCPredicate));
  commandExecutor->addCommand(walkNCCommand, walkNCPredicate, turnNCommandAndPredicate->getPreHandler());
  commandExecutor->addCommand(turnNCommandAndPredicate->getCommand(), turnNCommandAndPredicate->getPredicate(), new ExecutePreparationWhenExitBeforeCommandHandler(walkNPredicate));
  commandExecutor->addCommand(walkNCommand, walkNPredicate, doNothingHandler);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, doNothingHandler);
}
#endif

#if defined(UFORunnerTestMode)
void initializeCommandExecutor()
{
  // CommandExecutorの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // なにもしないハンドラ
  Handler *doNothingHandler = new Handler();

  // タッチセンサ待機コマンドの初期化とCommandExecutorへの追加
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, doNothingHandler); // なにもしないコマンドでタッチセンサがプレスされるのを待つ

  // UFO走行コマンドの初期化とCommandExecutorへの追加
  float n = 5.0;
  int walkPWM = 20;
  int turnPWM = 10;
  float swingLeft = 90.0;
  float swingRight = -90.0;
  int targetLeft = 30;
  int targetRight = 30;
  SwingSonarObstacleDetector *swingSonarObstacleDetector = new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, turnPWM, swingLeft, swingRight, targetLeft, targetRight);
  UFORunner *ufoRunner = new UFORunner(n, walkPWM, turnPWM, swingSonarObstacleDetector);
  commandExecutor->addCommand(ufoRunner, new FinishedCommandPredicate(ufoRunner), doNothingHandler);

  // 停止コマンドの初期化とCommandExecutorへの追加
  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, doNothingHandler);
}
#endif

void runner_task(intptr_t exinf)
{
  init_f(string("*** yamatea ***"));
  setting();              // 設定を済ませてから
  commandExecutor->run(); // 走らせる
  ext_tsk();
}

void main_task(intptr_t unused)
{
  const uint32_t duration = 100 * 1000;

  // commandExecutorを初期化する
  initializeCommandExecutor();

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
    clock->sleep(duration);
  }

  // メインタスクの終了
  ext_tsk();

  // オブジェクトの削除
  delete commandExecutor;
  delete robotAPI;
  delete touchSensor;
  delete colorSensor;
  delete sonarSensor;
  delete leftWheel;
  delete rightWheel;
  delete clock;
}
