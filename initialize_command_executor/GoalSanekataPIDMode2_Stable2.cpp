#include "Setting.h"
#ifdef GoalSanekataPIDMode2_Stable2

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

    float pmanDistance = 34;
    float carrotDistance = 32;
    float bananaDistance = 38;
    float peachDistance = 34;
    float orangeDistance = 74; // 73.5+-1
    float starFruitsDistance = 12;
    float cherryDistance = 60;
    float waterMelonDistance = 261; // 260前後。かなりブレあり
    float bokChoyDistance = 15;
    float dorianDistance = 40;
    float hassakuDistance = 35;
    float radishDistance = 31.5;
    float melonDistance = 34;
    float nutsDistance = 16.2;
    float lemonDistance = 38;
    float cucumberDistance = 191;  // 194+-3
    float strawberryDistance = 38; // 19;左車輪
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
    predicateStrawberry = predicateStrawberry->generateReversePredicate();
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
}
#endif