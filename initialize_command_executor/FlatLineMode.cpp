#include "Setting.h"
#ifdef FlatLineMode

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