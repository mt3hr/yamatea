#include "Setting.h"
#ifdef TrueCourceSanekataMode1261b

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
#ifdef TrueCourceSanekataModeCS
  // ↓ここから実方↓
  {
    // int orangePlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    int orangePlan = 2; // 弱めのPDでRに頼った走行

    // int orangePlan = 3;
    // int cherryPlan = 1; // 強めのPD、なるべく弱めのRな走行
    int cherryPlan = 2; // 弱めのPDでRに頼った走行
    // int cherryPlan = 3;// まあまあのPDでRに頼った走行
    // int waterMelonPlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    // int waterMelonPlan = 2; // 弱めのPDでRに頼った走行
    // int waterMelonPlan = 3; // まあまあのPDでRに頼った走行
    int waterMelonPlan = 4; // 弱めのPIDでRに頼った走行
    // int dorianPlan = 1; // まぁまぁなPID走行 iを使うと安定性が下がるのでplan2, plan3を用意しました
    int dorianPlan = 2; // 弱めのPD走行
    // int dorianPlan = 3; // 強めのPD走行

    // 電圧補正
    // SetPWMCoefficient *setPWMCoefficient = new SetPWMCoefficient();
    // commandExecutor->addCommand(setPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(setPWMCoefficient));

    bool facingAngleAtStarFruits = false;
    bool facingAngleAtBokChoy = false;

    // TODO 学校ではスターフルーツがだめっぽい

    float pmanDistance = 34;
    float carrotDistance = 32;
    float bananaDistance = 38;
    bananaDistance = 37; // CS
    float peachDistance = 34;
    float orangeDistance = 74;    // 73.5+-1
    orangeDistance = 73;          // CS
    float starFruitsDistance = 5; // CS 5 // 12;
    float cherryDistance = 60;
    float waterMelonDistance = 262.5; // 260前後。かなりブレあり
    waterMelonDistance = 263.5;       // 学校
    waterMelonDistance = 255.5;       // CS
    waterMelonDistance = 260.5;       // CS
    float bokChoyDistance = 15;
    bokChoyDistance = 20;
    float dorianDistance = 40;
    float hassakuDistance = 35;
    float radishDistance = 31.5;
    float melonDistance = 34;
    float nutsDistance = 16.2;
    nutsDistance = 13.5; // CS
    nutsDistance = 11.5; // CS
    float lemonDistance = 38;
    lemonDistance = 41;             // CS
    float cucumberDistance = 189.5; // 194+-3
    cucumberDistance = 185.5;       // CS
    cucumberDistance = 183;         // CS
    cucumberDistance = 180.5;       // CS
    cucumberDistance = 184;         // CS
    float strawberryDistance = 38;  // 19;左車輪
    strawberryDistance = 30;        // 19;左車輪
    strawberryDistance = 19;        // 19;左車輪
    strawberryDistance = 15;        // 19;左車輪
    strawberryDistance = 19;        // 左車輪にかけるか？
    float kiwiDistance = 40;
    float cabbageDistance = 50;

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
    pwm = 60;
    kp = 0.35;
    ki = 0.015;
    kd = kp * 3;
    dt = 0.4;
    r = 55;
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
      pwm = 60;
      kp = 0.4; // TODO
      ki = 0;
      kd = kp * 3;
      dt = 1;
      r = -34;
      r = -34.7; // 学校
      break;
    }
    case 3:
    {
      // TODO
      pwm = 60;
      kp = 0.675;
      ki = 0.01;
      kd = kp * 3;
      dt = 1;
      r = -34;
      break;
    }
    }
    angle = 10;
    PIDTracerV2 *orangePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
    // predicateOrange = new FacingRobotUseWheelPredicate(angle);
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
    radius = 34;
    radius = 28;  // 学校
    theta = -360; // 多めにしないと動かんのか？
    angle = 28;
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
      waterMelonPWM = 65;
      waterMelonKp = 0.4; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 45; // TODO
      waterMelonR = 46; // CS
      break;
    }
    case 2:
    {
      // 弱めのPDでRに頼った走行
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
      waterMelonPWM = 65;
      waterMelonKp = 0.4;
      waterMelonKi = 0.01;
      waterMelonKd = waterMelonKp * 3;
      waterMelonDt = 1;
      waterMelonR = 27; // TODO
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
    angle = 269;
    PIDTracerV2 *waterMelonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
    // predicateWaterMelon = new FacingRobotUseWheelPredicate(angle);
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
    pwm = 70;
    radius = 26;
    theta = -360; // 多めにしないと動かんのか？
    angle = -1;
    CurvatureWalkerCommandAndPredicate *bokChoyWalker = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
    Predicate *predicateBokChoy = new WheelDistancePredicate(bokChoyDistance, robotAPI);
    predicateBokChoy = predicateBokChoy->generateReversePredicate(); // 右車輪のほうが回転数多くなるのでそちらではかったほうが精度高くなりそう
    // predicateBokChoy = new FacingRobotUseWheelPredicate(angle);
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
    // 沖原あすぱらがすを流用
    // pwm = 30;
    // kp = 0.6;
    // ki = 0;
    // kd = 2.0;
    // dt = 1;
    // r = 0;
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
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 30;

    PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
    calibrator->addPIDTracer(melonPIDTracer);
    commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

    // nutsの初期化とCommandExecutorへの追加  ここから
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 4;
    PIDTracerV2 *nutsPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatenuts = new WheelDistancePredicate(nutsDistance, robotAPI);
    calibrator->addPIDTracer(nutsPIDTracer);
    commandExecutor->addCommand(nutsPIDTracer, predicatenuts, GET_VARIABLE_NAME(nutsPIDTracer));

    // LemonPIDTracerの初期化とCommandExecutorへの追加
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 30;

    PIDTracerV2 *lemonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateLemon = new WheelDistancePredicate(lemonDistance, robotAPI);
    calibrator->addPIDTracer(lemonPIDTracer);
    commandExecutor->addCommand(lemonPIDTracer, predicateLemon, GET_VARIABLE_NAME(lemonPIDTracer));

    // CucumberPIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.5;
    ki = 2.9;
    kd = 0.21;
    dt = 0.05;
    r = 0;
    PIDTracerV2 *cucumberPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
    calibrator->addPIDTracer(cucumberPIDTracer);
    commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

    // StrawberryPIDTracerの初期化とCommandExecutorへの追加
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = -25; // 44
    PIDTracerV2 *strawberryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
    // predicateStrawberry = predicateStrawberry->generateReversePredicate();
    calibrator->addPIDTracer(strawberryPIDTracer);
    commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

    // KiwiPIDTracerの初期化とCommandExecutorへの追加
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.5;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *kiwiPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateKiwi = new WheelDistancePredicate(kiwiDistance, robotAPI);
    calibrator->addPIDTracer(kiwiPIDTracer);
    commandExecutor->addCommand(kiwiPIDTracer, predicateKiwi, GET_VARIABLE_NAME(kiwiPIDTracer));

    // CabbagePIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.6;
    ki = 2.65;
    kd = 0.21;
    dt = 0.05;
    r = 0;
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.5;
    ki = 0;
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

#ifdef TrueCourceSanekataMode1261b
  {
    // int orangePlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    int orangePlan = 2; // 弱めのPDでRに頼った走行
    // int orangePlan = 3;
    // int cherryPlan = 1; // 強めのPD、なるべく弱めのRな走行
    int cherryPlan = 2; // 弱めのPDでRに頼った走行
    // int cherryPlan = 3;// まあまあのPDでRに頼った走行
    // int waterMelonPlan = 1; // 強めのPDとI、なるべく弱めのRな走行 ブレがあるのでplan2を用意しました
    // int waterMelonPlan = 2; // 弱めのPDでRに頼った走行
    // int waterMelonPlan = 3; // まあまあのPDでRに頼った走行
    int waterMelonPlan = 4; // 弱めのPIDでRに頼った走行
    // int dorianPlan = 1; // まぁまぁなPID走行 iを使うと安定性が下がるのでplan2, plan3を用意しました
    int dorianPlan = 2; // 弱めのPD走行
    // int dorianPlan = 3; // 強めのPD走行

    // 電圧補正
    // SetPWMCoefficient *setPWMCoefficient = new SetPWMCoefficient();
    // commandExecutor->addCommand(setPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(setPWMCoefficient));

    bool facingAngleAtStarFruits = false;
    bool facingAngleAtBokChoy = false;

    // TODO 学校ではスターフルーツがだめっぽい

    float pmanDistance = 34;
    float carrotDistance = 32;
    float bananaDistance = 38;
    bananaDistance = 37; // CS
    bananaDistance = 34; // 1261b
    float peachDistance = 34;
    peachDistance = 38;           // 1261b
    float orangeDistance = 74;    // 73.5+-1
    orangeDistance = 73;          // CS
    orangeDistance = 73;          // 1261b
    float starFruitsDistance = 5; // CS 5 // 12;
    starFruitsDistance = 12;      // 1261b
    float cherryDistance = 60;
    float waterMelonDistance = 262.5; // 260前後。かなりブレあり
    waterMelonDistance = 263.5;       // 学校
    waterMelonDistance = 255.5;       // CS
    waterMelonDistance = 260.5;       // CS
    waterMelonDistance = 261;         // 1261b
    waterMelonDistance = 263;         // 1261b
    float bokChoyDistance = 15;
    bokChoyDistance = 20;
    float dorianDistance = 40;
    float hassakuDistance = 35;
    hassakuDistance = 33;
    hassakuDistance = 31;
    float radishDistance = 31.5;
    float melonDistance = 34;
    float nutsDistance = 16.2;
    nutsDistance = 13.5; // CS
    nutsDistance = 11.5; // CS
    nutsDistance = 16.2; // 1261b
    float lemonDistance = 38;
    lemonDistance = 41;             // CS
    float cucumberDistance = 189.5; // 194+-3
    cucumberDistance = 185.5;       // CS
    cucumberDistance = 183;         // CS
    cucumberDistance = 180.5;       // CS
    cucumberDistance = 184;         // CS
    float strawberryDistance = 38;  // 19;左車輪
    strawberryDistance = 30;        // 19;左車輪
    strawberryDistance = 19;        // 19;左車輪
    strawberryDistance = 15;        // 19;左車輪
    strawberryDistance = 19;        // 左車輪にかけるか？
    float kiwiDistance = 40;
    float cabbageDistance = 50;

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
    pwm = 60;
    kp = 0.35;
    ki = 0.015;
    kd = kp * 3;
    dt = 0.4;
    r = 55;
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
      pwm = 60;
      kp = 0.4; // TODO
      ki = 0;
      kd = kp * 3;
      dt = 1;
      r = -34;
      r = -34.7; // 1261b
      break;
    }
    case 3:
    {
      // TODO
      pwm = 60;
      kp = 0.675;
      ki = 0.01;
      kd = kp * 3;
      dt = 1;
      r = -34;
      break;
    }
    }
    angle = 10;
    PIDTracerV2 *orangePIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateOrange = new WheelDistancePredicate(orangeDistance, robotAPI);
    // predicateOrange = new FacingRobotUseWheelPredicate(angle);
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
    radius = 34;
    radius = 28;  // 学校
    radius = 25;  // 1261b
    theta = -360; // 多めにしないと動かんのか？
    angle = 28;
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
      waterMelonPWM = 65;
      waterMelonKp = 0.4; // TODO
      waterMelonKi = 0;
      waterMelonKd = waterMelonKp * 3; // TODO
      waterMelonDt = 1;
      waterMelonR = 45; // TODO
      waterMelonR = 46; // CS
      break;
    }
    case 2:
    {
      // 弱めのPDでRに頼った走行
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
      waterMelonPWM = 65;
      waterMelonKp = 0.4;
      waterMelonKi = 0.01;
      waterMelonKd = waterMelonKp * 3;
      waterMelonDt = 1;
      waterMelonR = 27; // TODO
      waterMelonR = 28; // 1261b
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
    angle = 269;
    PIDTracerV2 *waterMelonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateWaterMelon = new WheelDistancePredicate(waterMelonDistance, robotAPI);
    // predicateWaterMelon = new FacingRobotUseWheelPredicate(angle);
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
    pwm = 70;
    radius = 26;
    radius = 24;  // 1261b
    theta = -360; // 多めにしないと動かんのか？
    angle = -1;
    CurvatureWalkerCommandAndPredicate *bokChoyWalker = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
    Predicate *predicateBokChoy = new WheelDistancePredicate(bokChoyDistance, robotAPI);
    predicateBokChoy = predicateBokChoy->generateReversePredicate(); // 右車輪のほうが回転数多くなるのでそちらではかったほうが精度高くなりそう
    // predicateBokChoy = new FacingRobotUseWheelPredicate(angle);
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
    // 沖原あすぱらがすを流用
    // pwm = 30;
    // kp = 0.6;
    // ki = 0;
    // kd = 2.0;
    // dt = 1;
    // r = 0;
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
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 30;

    PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
    calibrator->addPIDTracer(melonPIDTracer);
    commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

    // nutsの初期化とCommandExecutorへの追加  ここから
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 4;
    PIDTracerV2 *nutsPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatenuts = new WheelDistancePredicate(nutsDistance, robotAPI);
    calibrator->addPIDTracer(nutsPIDTracer);
    commandExecutor->addCommand(nutsPIDTracer, predicatenuts, GET_VARIABLE_NAME(nutsPIDTracer));

    // LemonPIDTracerの初期化とCommandExecutorへの追加
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 30;

    PIDTracerV2 *lemonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateLemon = new WheelDistancePredicate(lemonDistance, robotAPI);
    calibrator->addPIDTracer(lemonPIDTracer);
    commandExecutor->addCommand(lemonPIDTracer, predicateLemon, GET_VARIABLE_NAME(lemonPIDTracer));

    // CucumberPIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.5;
    ki = 2.9;
    kd = 0.21;
    dt = 0.05;
    r = 0;
    PIDTracerV2 *cucumberPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
    calibrator->addPIDTracer(cucumberPIDTracer);
    commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

    // StrawberryPIDTracerの初期化とCommandExecutorへの追加
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.35;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = -25; // 44
    PIDTracerV2 *strawberryPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateStrawberry = new WheelDistancePredicate(strawberryDistance, robotAPI);
    // predicateStrawberry = predicateStrawberry->generateReversePredicate();
    calibrator->addPIDTracer(strawberryPIDTracer);
    commandExecutor->addCommand(strawberryPIDTracer, predicateStrawberry, GET_VARIABLE_NAME(strawberryPIDTracer));

    // KiwiPIDTracerの初期化とCommandExecutorへの追加
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.5;
    ki = 0;
    kd = kp * 3;
    dt = 1;
    r = 0;
    PIDTracerV2 *kiwiPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateKiwi = new WheelDistancePredicate(kiwiDistance, robotAPI);
    calibrator->addPIDTracer(kiwiPIDTracer);
    commandExecutor->addCommand(kiwiPIDTracer, predicateKiwi, GET_VARIABLE_NAME(kiwiPIDTracer));

    // CabbagePIDTracerの初期化とCommandExecutorへの追加
    pwm = 65;
    kp = 0.6;
    ki = 2.65;
    kd = 0.21;
    dt = 0.05;
    r = 0;
    // はっさくを流用する
    pwm = 25;
    pwm = 40; // 動くかな
    kp = 0.5;
    ki = 0;
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

#endif

#ifdef TrueCourceOkiharaModeCSForKomichiRobot
  // ↓ここから沖原↓
  {
    SetPWMCoefficient *setPWMCoefficient = new SetPWMCoefficient();
    commandExecutor->addCommand(setPWMCoefficient, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(setPWMCoefficient));

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
    int sceneBananaMotorCountPredicateArg = 1200;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
    int scenePeachMotorCountPredicateArg = 1540;      // バナナとオレンジの間の小さいカーブ
    int sceneOrangeMotorCountPredicateArg = 2450;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
    int sceneStarFruitsMotorCountPredicateArg = 2540; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
    int sceneCherryMotorCountPredicateArg = 2750;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
    sceneCherryMotorCountPredicateArg = 2800;         // 小路ロボ     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
    int sceneBokChoyMotorCountPredicateArg = 6300;    // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
    sceneBokChoyMotorCountPredicateArg = 6350;
    int sceneDorianMotorCountPredicateArg = 6600;    // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneAsparagusMotorCountPredicateArg = 7100; // ドリアン終了後の１つ目の直線
    int sceneRadishMotorCountPredicateArg = 7440;    // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    // int sceneRadishMotorCountPredicateArg = 7490;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    int sceneMelonMotorCountPredicateArg = 8040; // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
    int sceneNutsMotorCountPredicateArg = 8300;
    int sceneLemonMotorCountPredicateArg = 8690 + 50;       // 8690;
    int sceneCucumberMotorCountPredicateArg = 10605 + 60;   // 10605;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
    int sceneStrawberryMotorCountPredicateArg = 11125 - 80; // 11125; // ゴールまで。いちご好き。ライントレースする。
    int sceneCabbageMotorCountpredicateArg = 12000;         // ゴールまで。

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
    int nutsDistance = (sceneNutsMotorCountPredicateArg) / (360 / (wheelDiameter * M_PI)) - distanceTemp;
    distanceTemp += nutsDistance;
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
    float dt;
    float r;
    float leftPow;
    float rightPow;

    // CarrotPIDTracerの初期化とCommandExecutorへの追加
    // 下記コメントアウト箇所アンパイ

    pwm = 25;
    kp = 0.9;
    ki = 0.06;
    ki = 0.01;
    kd = 1.4;
    dt = 1;
    r = 0;

    /*
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 1.5;
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

    /*
    pwm = 25;
    kp = 0.8;
    ki = 0.05;
    kd = 1.8;
    dt = 1;
    r = 0;
    */

    pwm = 30;
    kp = 1.2;
    ki = 0;
    ki = 0.01;
    kd = 2.0;
    // kd = kp * 3;
    dt = 1;
    r = 0;

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

    /*沖原ロボ用の値
    pwm = 40;
    kp = 1.1;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    pwm = 40;
    kp = 1.0;
    ki = 0.01; // okihara 0;
    kd = 1.8;
    // kd = kp * 3;
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

    /*沖原ロボ用の値
    pwm = 40;
    kp = 0.7;
    ki = 0;
    kd = 2.2;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    pwm = 40;
    kp = 0.68; // okihara 0.6;
    ki = 0;
    // ki = 0.01;
    kd = 2.13; // okihara 2.1;
    // kd = kp * 3;
    dt = 1;
    r = 0;

    // 下記はアンパイ
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
    leftPow = 40;
    rightPow = 44;

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

    /* 沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.1;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    pwm = 30;
    kp = 1.1;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;

    /*
    pwm = 25;
    kp = 0.9;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
    calibrator->addPIDTracer(melonPIDTracer);
    commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

    pwm = 30;
    kp = 0.45;
    ki = 0.01;
    kd = 1.5;
    dt = 1;
    r = 0;
    PIDTracerV2 *nutsPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateNuts = new WheelDistancePredicate(nutsDistance, robotAPI);
    calibrator->addPIDTracer(nutsPIDTracer);
    commandExecutor->addCommand(nutsPIDTracer, predicateNuts, GET_VARIABLE_NAME(nutsPIDTracer));

    // LemonPIDTracerの初期化とCommandExecutorへの追加

    /*
    沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.1;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    pwm = 30;
    kp = 0.93; // 1;
    ki = 0;
    kd = 2.3; // okihara 2.2;
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

    /*沖原ロボ用の値
    pwm = 60;
    kp = 1.0;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    pwm = 60;
    kp = 0.8;
    ki = 0.01; // okihara 0;
    kd = 2.4;  // okihara 1.5;
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

    /*沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    ki = 0.01;
    kd = 3; // okihara 2.5;
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
    kp = 0.8;
    ki = 0;
    kd = 2.0; // 1.5;
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
#ifdef TrueCourceOkiharaModeCSUseRegionalValueKomichiRobot
  // ↓ここから沖原↓
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

    int pwm;
    float kp;
    float ki;
    float kd;
    float dt;

    float leftPow;
    float rightPow;

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
#ifdef TrueCourceOkiharaModeCSStableValueForOkiharaRobot
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
    int sceneBananaMotorCountPredicateArg = 1200;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
    int scenePeachMotorCountPredicateArg = 1540;      // バナナとオレンジの間の小さいカーブ
    int sceneOrangeMotorCountPredicateArg = 2450;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
    int sceneStarFruitsMotorCountPredicateArg = 2540; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
    int sceneCherryMotorCountPredicateArg = 2750;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
    int sceneBokChoyMotorCountPredicateArg = 6300;    // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
    int sceneDorianMotorCountPredicateArg = 6600;     // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneAsparagusMotorCountPredicateArg = 7100;  // ドリアン終了後の１つ目の直線
    int sceneRadishMotorCountPredicateArg = 7440;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    // int sceneRadishMotorCountPredicateArg = 7490;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    int sceneMelonMotorCountPredicateArg = 8040; // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
    int sceneLemonMotorCountPredicateArg = 8690;
    int sceneCucumberMotorCountPredicateArg = 10605;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
    int sceneStrawberryMotorCountPredicateArg = 11125; // ゴールまで。いちご好き。ライントレースする。
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
    // 下記コメントアウト箇所アンパイ

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
    kd = 1.5;
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
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    PIDTracerV2 *peachPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicatePeach = new WheelDistancePredicate(peachDistance, robotAPI);
    calibrator->addPIDTracer(peachPIDTracer);
    commandExecutor->addCommand(peachPIDTracer, predicatePeach, GET_VARIABLE_NAME(peachPIDTracer));

    // OrangePIDTracerの初期化とCommandExecutorへの追加

    pwm = 30;
    kp = 0.7;
    ki = 0.01;
    kd = 1.8;
    dt = 1;
    r = 0;

    /*沖原ロボ用の値
    pwm = 40;
    kp = 1.1;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    /*
    pwm = 40;
    kp = 1.0;
    ki = 0;
    kd = 1.8;
    dt = 1;
    r = 0;
    */

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

    /*沖原ロボ用の値
    pwm = 40;
    kp = 0.7;
    ki = 0;
    kd = 2.2;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    /*
    pwm = 40;
    kp = 0.6;
    ki = 0;
    kd = 2.1;
    dt = 1;
    r = 0;
    */

    // 下記はアンパイ
    pwm = 30;
    kp = 0.7;
    ki = 0;
    kd = 1.5;
    dt = 1;
    r = 0;

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

    /*
    pwm = 30;
    kp = 0.6;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    pwm = 25;
    kp = 0.65;
    ki = 0;
    kd = 1.2;
    dt = 1;
    r = 0;

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

    /* 沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.1;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    /*
    pwm = 30;
    kp = 1.1;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    pwm = 25;
    kp = 0.9;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;

    PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
    calibrator->addPIDTracer(melonPIDTracer);
    commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

    // LemonPIDTracerの初期化とCommandExecutorへの追加

    /*
    沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.1;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    /*
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.2;
    dt = 1;
    r = 0;
    */

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

    /*沖原ロボ用の値
    pwm = 60;
    kp = 1.0;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    pwm = 60;
    kp = 0.8;
    ki = 0;
    kd = 1.5;
    dt = 1;
    r = 0;

    PIDTracerV2 *cucumberPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateCucumber = new WheelDistancePredicate(cucumberDistance, robotAPI);
    calibrator->addPIDTracer(cucumberPIDTracer);
    commandExecutor->addCommand(cucumberPIDTracer, predicateCucumber, GET_VARIABLE_NAME(cucumberPIDTracer));

    // StrawberryPIDTracerの初期化とCommandExecutorへの追加

    pwm = 25;
    kp = 0.7;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;

    /*沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;
    */

    // こみちロボ用の値
    /*
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;
    */

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
    kp = 0.8;
    ki = 0;
    kd = 1.5;
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
#ifdef TrueCourceOkiharaModeCSForOkiharaRobot
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
    int sceneBananaMotorCountPredicateArg = 1200;     // 8の字急カーブ突入前。バナナっぽい形しているので。ライントレースする。
    int scenePeachMotorCountPredicateArg = 1540;      // バナナとオレンジの間の小さいカーブ
    int sceneOrangeMotorCountPredicateArg = 2450;     // 8の字クロス1回目突入前。オレンジぐらいの大きさの円形なので（え？）。安定しないのでpwm弱めでライントレースする。
    int sceneStarFruitsMotorCountPredicateArg = 2540; // 8の字クロス1回目通過後。十字っぽい果物や野菜といったらスターフルーツなので。シナリオトレースで左弱めの直進をする。
    int sceneCherryMotorCountPredicateArg = 2750;     // 8の字クロス1回目通過後ライントレース復帰時。さくらんぼくらい小さいので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneWaterMelonMotorCountPredicateArg = 6170; // 8の字クロス2回目突入前。メロンぐらいでかいので。ライントレースする。
    int sceneBokChoyMotorCountPredicateArg = 6300;    // 8の時クロス2回目通過後直進中。青梗菜も上から見たら十字っぽいので（？）。シナリオトレースで直進する。
    int sceneDorianMotorCountPredicateArg = 6600;     // 8の字クロス2回目通過後ライントレース復帰時。ドリアンぐらい臭い（処理的に怪しい）ので。ラインに戻るためにpwm弱めでライントレースする。
    int sceneAsparagusMotorCountPredicateArg = 7100;  // ドリアン終了後の１つ目の直線
    int sceneRadishMotorCountPredicateArg = 7440;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    // int sceneRadishMotorCountPredicateArg = 7490;     // アスパラガス終了後メロンのカーブ手前までの２つ目の直線
    int sceneMelonMotorCountPredicateArg = 8040; // 中央直進突入後。カットされたメロンみたいな形して　いねーよな。ライントレースする。
    int sceneLemonMotorCountPredicateArg = 8690;
    int sceneCucumberMotorCountPredicateArg = 10655;   // 中央直進脱出前。きゅうりぐらいまっすぐな心を持ちたい。直視なのでpwm強めでライントレースする。
    int sceneStrawberryMotorCountPredicateArg = 11125; // ゴールまで。いちご好き。ライントレースする。
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
    // 下記コメントアウト箇所アンパイ

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
    kd = 1.5;
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

    /*
    pwm = 25;
    kp = 0.8;
    ki = 0.05;
    kd = 1.8;
    dt = 1;
    r = 0;
    */

    pwm = 30;
    kp = 1.2;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;

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

    // 沖原ロボ用の値
    pwm = 40;
    kp = 1.1;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;

    // こみちロボ用の値
    /*
    pwm = 40;
    kp = 0.8;
    ki = 0;
    kd = 1.8;
    dt = 1;
    r = 0;
    */

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

    // 沖原ロボ用の値
    pwm = 40;
    kp = 0.7;
    ki = 0;
    kd = 2.2;
    dt = 1;
    r = 0;

    // こみちロボ用の値
    /*
    pwm = 40;
    kp = 0.6;
    ki = 0;
    kd = 2.1;
    dt = 1;
    r = 0;
    */

    // 下記はアンパイ
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

    // 沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.1;
    dt = 1;
    r = 0;

    // こみちロボ用の値
    /*
    pwm = 30;
    kp = 1.1;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    /*
    pwm = 25;
    kp = 0.9;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;
    */

    PIDTracerV2 *melonPIDTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
    Predicate *predicateMelon = new WheelDistancePredicate(melonDistance, robotAPI);
    calibrator->addPIDTracer(melonPIDTracer);
    commandExecutor->addCommand(melonPIDTracer, predicateMelon, GET_VARIABLE_NAME(melonPIDTracer));

    // LemonPIDTracerの初期化とCommandExecutorへの追加

    // 沖原ロボ用の値
    pwm = 30;
    kp = 1.2;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;

    // こみちロボ用の値
    /*
    pwm = 30;
    kp = 0.9;
    ki = 0;
    kd = 2.2;
    dt = 1;
    r = 0;
    */

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

    // 沖原ロボ用の値
    pwm = 60;
    kp = 1.0;
    ki = 0;
    kd = 2.0;
    dt = 1;
    r = 0;

    // 小路ロボ用
    /*
    pwm = 60;
    kp = 0.8;
    ki = 0;
    kd = 1.5;
    dt = 1;
    r = 0;
    */

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

    // 沖原ロボ用の値
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;

    // こみちロボ用の値
    /*
    pwm = 30;
    kp = 1.0;
    ki = 0;
    kd = 2.5;
    dt = 1;
    r = 0;
    */

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

      // PIDトレースで少し進む
      distance = 20;
      Predicate *distancePredicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(lowPWMTracer, distancePredicate, GET_VARIABLE_NAME(lowPWMTracer));

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

      numberOfTime = 45;
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
      numberOfTime = 17;
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
      pwm = -7;
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
      distance = 3.2;
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
      distance = 8;
      pwm = 7;
      PIDStraightWalker *walker5 = new PIDStraightWalker(pwm, straightKp, straightKi, straightKd, straightDt);
      Predicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
      commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));
#endif
#ifdef SlalomPattern2
      distance = 1.5; // 4;
      pwm = 7;
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
      radius = 10.8; // 11.5;
      radius = 10.3; // 1261b
      theta = 50;
      CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
      commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      pwm = 5 * coefficientPWMForCurve;
      radius = 10.8; // 11.5;
      radius = 10.3; // 1261b
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
      int diff = 3;
      // pwm = 10 * coefficientPWM;
      pwm = 7 * coefficientPWM;
      distance = 3 + diff;
      HedgehogUsePID *headgehog2 = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehog2, new FinishedCommandPredicate(headgehog2), GET_VARIABLE_NAME(headgehog2));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // 向き調節
      angle = 25;
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
      distance = 14 + diff;
      HedgehogUsePID *headgehogAA = new HedgehogUsePID(distance, pwm, straightKp, straightKi, straightKd, straightDt);
      commandExecutor->addCommand(headgehogAA, new FinishedCommandPredicate(headgehogAA), GET_VARIABLE_NAME(headgehogAA));
      commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

      // カーブ
      radius = 12.5;
      radius = 12; // simulator
      theta = -90;
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
      radius = 35.5;
      theta = 30;
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
      distance = 1;
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
    // int *r = new int(25);
    // int *g = new int(30);
    // int *b = new int(40);
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
    // 指定距離IPD
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
    // 青の後ちょっと進む
    //   Predicate *predicate10bl = new WheelDistancePredicate(10, robotAPI);
    //    commandExecutor->addCommand(walker10, predicate10bl, GET_VARIABLE_NAME(walker10));
    Stopper *stopper10 = new Stopper();
    Predicate *predicateS10 = new NumberOfTimesPredicate(1);
    commandExecutor->addCommand(stopper10, predicateS10, GET_VARIABLE_NAME(stoppper10));
    // １１ 指定角度右回転（青ラインに向く）
    // CommandAndPredicate *predicate11 = new RotateRobotUseGyroCommandAndPredicate(0, 5, robotAPI);
    // commandExecutor->addCommand(predicate11->getCommand(), predicate11->getPredicate(), GET_VARIABLE_NAME(predicate11->getCommand()));
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
      { // ジャイロで調節するならtrueに
        CommandAndPredicate *predicate13 = new RotateRobotUseGyroCommandAndPredicate(45, 5, robotAPI);
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
      Predicate *predicate13blue = new WheelDistancePredicate(15, robotAPI);
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