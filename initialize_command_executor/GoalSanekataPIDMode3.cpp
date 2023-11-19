#include "Setting.h"
#ifdef GoalSanekataPIDMode3

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
  // ColorReader *colorReader = new ColorReader();
  // colorid_t *garageCardColorPtr = colorReader->getColorPtr();

  {
    // SetPWMCoefficient *setPWMCoefficient = new SetPWMCoefficient();
    // commandExecutor->addCommand(setPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(setPWMCoefficient));

    float pmanDistance = 34;
    float carrotDistance = 31;
    float bananaDistance = 37;
    float peachDistance = 31;
    float orangeDistance = 72;
    float starFruitsDistance = 5;
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
    float dt;
    float kp;
    float ki;
    float kd;
    float tu;
    float ku;
    float r;
    float radius;
    float theta;
    float angle;

    // にんじん
    float carrotDt = 0.05;
    float carrotPWM = 45;
    float carrotKu = 2.35; // 1.8;
    float carrotTu = 1.85; // 1.3293;
    float carrotR = 0;     // 28 * 1 / 5;

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
    kd = 1.1;
    dt = 1;
    r = 0;
    PIDTracerV2 *bananaPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateBanana = new WheelDistancePredicate(bananaDistance, robotAPI);
    commandExecutor->addCommand(bananaPIDTracer, predicateBanana, GET_VARIABLE_NAME(bananaPIDTracer));
    calibrator->addPIDTracer(bananaPIDTracer);

    // PeachPIDTracerの初期化とCommandExecutorへの追加
    dt = carrotDt;
    pwm = carrotPWM;
    ku = carrotKu;
    tu = carrotTu;
    r = carrotR;
    PIDLimTracer *peachPIDTracer = new PIDLimTracer(RIGHT_TRACE, PLTM_PID_MOD, pwm, ku, tu, dt, r);
    Predicate *predicatePeach = new WheelDistancePredicate(peachDistance, robotAPI);
    calibrator->addPIDTracer(peachPIDTracer);
    commandExecutor->addCommand(peachPIDTracer, predicatePeach, GET_VARIABLE_NAME(peachPIDTracer));

    // OrangePIDTracerの初期化とCommandExecutorへの追加
    dt = 0.05;
    pwm = 45;
    ku = 2.5;  // 1.8
    tu = 1.45; // 1.3293
    r = 28 * 1.8 / 5;
    PIDLimTracer *orangePIDTracer = new PIDLimTracer(RIGHT_TRACE, PLTM_PID_MOD, pwm, ku, tu, dt, r);
    Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
    calibrator->addPIDTracer(orangePIDTracer);
    commandExecutor->addCommand(orangePIDTracer, predicateOrange, GET_VARIABLE_NAME(orangePIDTracer));

    // StarFruitsWalkerの初期化とCommandExecutorへの追加
    pwm = 60;
    radius = 22;
    theta = -360; // 多めにしないと動かんのか？
    angle = 24;
    CurvatureWalkerCommandAndPredicate *starFuitsWalker = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
    Predicate *predicateStarFruits = new WheelDistancePredicate(starFruitsDistance, robotAPI);
    // Predicate *predicateStarFruits = new FacingRobotUseWheelPredicate(angle);
    // Predicate *predicateStarFruits = new GyroRotateAnglePredicate(angle);
    predicateStarFruits = predicateStarFruits->generateReversePredicate(); // 右車輪のほうが回転数多くなるのでそちらではかったほうが精度高くなりそう
    commandExecutor->addCommand(starFuitsWalker->getCommand(), predicateStarFruits, GET_VARIABLE_NAME(starFuitsWalker));

    dt = 0.05;
    pwm = 55;
    ku = 2.8;
    tu = 1.3293;
    r = 25 * 0.5 / 5;
    PIDLimTracer *cherryPIDTracer = new PIDLimTracer(RIGHT_TRACE, PLTM_PID_MOD, pwm, ku, tu, dt, r);
    Predicate *predicateCherry = new WheelDistancePredicate(cherryDistance, robotAPI);
    calibrator->addPIDTracer(cherryPIDTracer);
    commandExecutor->addCommand(cherryPIDTracer, predicateCherry, GET_VARIABLE_NAME(cherryPIDTracer));

    // WaterMelonPIDTracerの初期化とCommandExecutorへの追加
    dt = 0.05;
    pwm = 55;
    ku = 2.8;
    tu = 1.3293;
    r = 25 * 0.5 / 5;
    PIDLimTracer *waterMelonPIDTracer = new PIDLimTracer(RIGHT_TRACE, PLTM_PID_MOD, pwm, ku, tu, dt, r);
    Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
    calibrator->addPIDTracer(waterMelonPIDTracer);
    commandExecutor->addCommand(waterMelonPIDTracer, predicateWaterMelon, GET_VARIABLE_NAME(waterMelonPIDTracer));

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
    dorianPWM = 35;
    dorianKp = 0.48;
    dorianKi = 0;
    dorianKd = kp * 3;
    dorianDt = 1;
    dorianR = 0;
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
    dt = carrotDt;
    pwm = carrotPWM;
    ku = carrotKu;
    tu = carrotTu;
    r = carrotR;
    PIDLimTracer *melonPIDTracer = new PIDLimTracer(RIGHT_TRACE, PLTM_PID_MOD, pwm, ku, tu, dt, r);
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
    dt = carrotDt;
    pwm = carrotPWM;
    ku = carrotKu;
    tu = carrotTu;
    r = carrotR;
    PIDLimTracer *lemonPIDTracer = new PIDLimTracer(RIGHT_TRACE, PLTM_PID_MOD, pwm, ku, tu, dt, r);
    Predicate *predicateLemon = new WheelDistancePredicate(lemonDistance, robotAPI);
    calibrator->addPIDTracer(lemonPIDTracer);
    commandExecutor->addCommand(lemonPIDTracer, predicateLemon, GET_VARIABLE_NAME(lemonPIDTracer));

    // CucumberPIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.6;
    ki = 2.65;
    kd = 0.21;
    dt = 0.05;
    r = 0;

    PIDTracerV2 *cucumberPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
    calibrator->addPIDTracer(cucumberPIDTracer);
    commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

    // StrawberryPIDTracerの初期化とCommandExecutorへの追加
    dt = carrotDt;
    pwm = carrotPWM;
    ku = carrotKu;
    tu = carrotTu;
    r = -carrotR;
    PIDLimTracer *strawberryPIDTracer = new PIDLimTracer(RIGHT_TRACE, PLTM_PID_MOD, pwm, ku, tu, dt, r);
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