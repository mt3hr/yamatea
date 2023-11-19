#include "Setting.h"
#ifdef SlalomAwaitingSignalPlan1SanekataMode

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
    float slalomAngle = 0; // 多分270
    // ガレージカードの色取得用ColorReader
    ColorReader *colorReader = new ColorReader();

    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;

    float r;
    float theta;

    float angle;

    float leftPWM;
    float rightPWM;

    int numberOfTime;

    Stopper *stopper = new Stopper();
    Predicate *stopperPredicate = new NumberOfTimesPredicate(1);

    // Calibrator
    PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
    Predicate *startButtonPredicate = new StartButtonPredicate();
    commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

    // スラローム進入ここから
    // コース上2つ目の青線前から開始。

    pwm = 15;
    kp = 0.2;
    ki = 0.1;
    kd = 0.2;
    dt = 1;
    ColorPIDTracer *pidTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
    pwm = 8;
    ColorPIDTracer *lowPWMTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
    calibrator->addColorPIDTracer(pidTracer);
    calibrator->addColorPIDTracer(lowPWMTracer);

#ifdef SimulatorMode
    rgb_raw_t targetRGB;
    targetRGB.r = 110;
    targetRGB.g = 100;
    targetRGB.b = 150;
    pidTracer->setTargetColor(targetRGB);
    lowPWMTracer->setTargetColor(targetRGB);
#endif

    // PIDトレースで青線まで進む
    // Predicate *pidTracerPredicate = new ColorPredicate(COLOR_BLUE);
    // commandExecutor->addCommand(pidTracer, pidTracerPredicate, GET_VARIABLE_NAME(pidTracer));

    // スラローム直前までPIDトレース
    float distance = 30;
    // commandExecutor->addCommand(pidTracer, new WheelDistancePredicate(distance, robotAPI), GET_VARIABLE_NAME(pidTracer));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // アームを下げる
    float armAngle = 15;
    pwm = -10;
    numberOfTime = 25;
    Command *armDown = new ArmController(pwm);
    Predicate *armDownPredicate = new NumberOfTimesPredicate(numberOfTime);
    commandExecutor->addCommand(armDown, armDownPredicate, GET_VARIABLE_NAME(armDown));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

// テールモータで角度をつける
#ifdef SimulatorMode
    pwm = 25;
#else
    pwm = 100;
#endif
    numberOfTime = 20;
    Command *tailMotorDrive = new TailController(pwm);
    Predicate *tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
    commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

// スラローム位置補正。アームを下げたまま直進。
#ifdef SimulatorMode
    numberOfTime = 50;
#else
    numberOfTime = 50;
#endif
    leftPWM = 5;
    rightPWM = 5;
    Command *lowWalker = new Walker(leftPWM, rightPWM);
    commandExecutor->addCommand(lowWalker, new NumberOfTimesPredicate(numberOfTime), GET_VARIABLE_NAME(lowWalker));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // ちょっとバックする（これがないとアームが引っかかって位置ずれする）
    Command *back = new Walker(-5, -5);
    distance = -3;
    Predicate *backPredicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(back, backPredicate, GET_VARIABLE_NAME(back));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // アームを戻す
    pwm = 10;
    Command *armUp = new ArmController(pwm);
    Predicate *armUpPredicate = new MotorRotateAnglePredicate(armAngle, robotAPI->getArmMotor());
    commandExecutor->addCommand(armUp, armUpPredicate, GET_VARIABLE_NAME(armUp));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // テールモータを戻す
#ifdef SimulatorMode
    pwm = 25;
#else
    pwm = 100;
#endif
    numberOfTime = 11;
    tailMotorDrive = new TailController(-pwm);
    tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
    commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // Walkerで少し進んでスラロームに進入する（PIDTracerだとベニヤ板の暗さで行けねえ）
    leftPWM = 10;
    rightPWM = 10;
    distance = 25;
    Walker *walkerC = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walkerCPredicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walkerC, walkerCPredicate, GET_VARIABLE_NAME(walkerC));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // テールモータを戻す
#ifdef SimulatorMode
    pwm = 25;
#else
    pwm = 100;
#endif
    numberOfTime = 10;
    tailMotorDrive = new TailController(-pwm);
    tailMotorDrivePreicate = new NumberOfTimesPredicate(numberOfTime);
    commandExecutor->addCommand(tailMotorDrive, tailMotorDrivePreicate, GET_VARIABLE_NAME(tailMotorDrive));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // 角度を調整する
    pwm = 5;
    FacingAngleAbs *facingAngle = new FacingAngleAbs(FA_Gyro, pwm, slalomAngle);
    commandExecutor->addCommand(facingAngle, new FinishedCommandPredicate(facingAngle), GET_VARIABLE_NAME(FacingAngle));

    // スラローム進入ここまで

    // 指示待ち走行ここから

    // 45度左旋回
    pwm = 5;
    angle = -45;
    RotateRobotUseGyroCommandAndPredicate *rotate1 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotate1->getCommand(), rotate1->getPredicate(), GET_VARIABLE_NAME(rotate1));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // 直進
    leftPWM = 10;
    rightPWM = 10;
    distance = 11.5;
    Walker *walker2 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker2Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker2, walker2Predicate, GET_VARIABLE_NAME(walker2));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // 45度右旋回
    pwm = 5;
    angle = 45;
    RotateRobotUseGyroCommandAndPredicate *rotate2 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotate2->getCommand(), rotate2->getPredicate(), GET_VARIABLE_NAME(rotate2));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // 角度を調整する
    pwm = 7;
    FacingAngleAbs *facingAngleAbs = new FacingAngleAbs(FA_Gyro, pwm, slalomAngle);
    commandExecutor->addCommand(facingAngleAbs, new FinishedCommandPredicate(facingAngleAbs), GET_VARIABLE_NAME(FacingAngleAbs));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 10;
    rightPWM = 10;
#endif
    distance = 20;
    Walker *walker3 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker3Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker3, walker3Predicate, GET_VARIABLE_NAME(walker3));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // カーブ
#ifdef SimulatorMode
    pwm = 20;
#else
    pwm = 7;
#endif
    r = 18;
    theta = 30;
    CurvatureWalkerCommandAndPredicate *curve1 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
    commandExecutor->addCommand(curve1->getCommand(), curve1->getPredicate(), GET_VARIABLE_NAME(curve1));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 11;
    rightPWM = 11;
#endif
    distance = 8;
    Walker *walker4 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker4Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker4, walker4Predicate, GET_VARIABLE_NAME(walker4));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // 角度を調整する
    pwm = 7;
    facingAngleAbs = new FacingAngleAbs(FA_WheelCount, pwm, slalomAngle);
    commandExecutor->addCommand(facingAngle, new FinishedCommandPredicate(facingAngle), GET_VARIABLE_NAME(FacingAngle));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 7;
    rightPWM = 7;
#endif
    distance = 14;
    Walker *walkerB = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walkerBPredicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walkerB, walkerBPredicate, GET_VARIABLE_NAME(walkerB));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    // 45度左旋回
#ifdef SimulatorMode
    pwm = 10;
#else
    pwm = 6;
#endif
    angle = -45;
    RotateRobotUseGyroCommandAndPredicate *rotateA = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotateA->getCommand(), rotateA->getPredicate(), GET_VARIABLE_NAME(rotateA));
    commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 7;
    rightPWM = 7;
#endif
    distance = 0.5;
    Walker *walker5 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker5Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker5, walker5Predicate, GET_VARIABLE_NAME(walker5));

    // カーブ
#ifdef SimulatorMode
    pwm = 20;
#else
    pwm = 8;
#endif
    r = 16;
    theta = 50;
    CurvatureWalkerCommandAndPredicate *curve2 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
    commandExecutor->addCommand(curve2->getCommand(), curve2->getPredicate(), GET_VARIABLE_NAME(curve2));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 7;
    rightPWM = 7;
#endif
    distance = 5;
    Walker *walkerQ = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walkerQPredicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walkerQ, walkerQPredicate, GET_VARIABLE_NAME(walkerQ));

    // カーブ
#ifdef SimulatorMode
    pwm = 20;
#else
    pwm = 8;
#endif
    r = 8;
    theta = 50;
    CurvatureWalkerCommandAndPredicate *curve3 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
    commandExecutor->addCommand(curve3->getCommand(), curve3->getPredicate(), GET_VARIABLE_NAME(curve3));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 7;
    rightPWM = 7;
#endif
    distance = 10;
    Walker *walker6 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker6Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker6, walker6Predicate, GET_VARIABLE_NAME(walker6));

    // カーブ
#ifdef SimulatorMode
    pwm = 20;
#else
    pwm = 10;
#endif
    r = 10;
    theta = -45;
    CurvatureWalkerCommandAndPredicate *curve4 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
    commandExecutor->addCommand(curve4->getCommand(), curve4->getPredicate(), GET_VARIABLE_NAME(curve4));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 7;
    rightPWM = 7;
#endif
    distance = 18;
    Walker *walker7 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker7Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker7, walker7Predicate, GET_VARIABLE_NAME(walker7));

    // ガレージカード色取得
    commandExecutor->addCommand(colorReader, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(colorReader));

#ifdef SlalomAwaitingSignalModePattern1_1
    // 85度左旋回
    pwm = 10;
    angle = -85;
    RotateRobotUseGyroCommandAndPredicate *rotate4 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotate4->getCommand(), rotate4->getPredicate(), GET_VARIABLE_NAME(rotate4));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 7;
    rightPWM = 7;
#endif
    distance = 33;
    Walker *walker8 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker8Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker8, walker8Predicate, GET_VARIABLE_NAME(walker8));

    // 90度左旋回
#ifdef SimulatorMode
    pwm = 10;
#else
    pwm = 10;
#endif
    angle = -90;
    RotateRobotUseGyroCommandAndPredicate *rotate5 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotate5->getCommand(), rotate5->getPredicate(), GET_VARIABLE_NAME(rotate5));

    // カーブ
#ifdef SimulatorMode
    pwm = 20;
#else
    pwm = 7;
#endif
    r = 13.5;
    theta = 90;
    CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
    commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));

    //  直進
#ifdef SimulatorMode
    leftPWM = 15;
    rightPWM = 15;
#else
    leftPWM = 7;
    rightPWM = 7;
#endif
    distance = 20;
    Walker *walker10 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker10Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker10, walker10Predicate, GET_VARIABLE_NAME(walker10));

    // 45度右旋回
    pwm = 10;
    angle = 45;
    RotateRobotUseGyroCommandAndPredicate *rotate6 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotate6->getCommand(), rotate6->getPredicate(), GET_VARIABLE_NAME(rotate6));

    //  直進。黒を拾うまで
    leftPWM = 10;
    rightPWM = 10;
    distance = 15;
    Walker *walker9 = new Walker(leftPWM, rightPWM);
    Predicate *walker9Predicate = new ColorPredicate(COLOR_BLACK);
    commandExecutor->addCommand(walker9, walker9Predicate, GET_VARIABLE_NAME(walker9));

    // PIDで青線まで進む
    commandExecutor->addCommand(lowPWMTracer, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(lowPWMTracer));

#endif

#ifdef SlalomAwaitingSignalModePattern2_1
    // 90度左回転
    pwm = 10;
    angle = -90;
    RotateRobotUseGyroCommandAndPredicate *rotate4 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotate4->getCommand(), rotate4->getPredicate(), GET_VARIABLE_NAME(rotate4));

    // 30度左回転
    pwm = 10;
    angle = -30;
    RotateRobotUseGyroCommandAndPredicate *rotate5 = new RotateRobotUseGyroCommandAndPredicate(angle, pwm, robotAPI);
    commandExecutor->addCommand(rotate5->getCommand(), rotate5->getPredicate(), GET_VARIABLE_NAME(rotate4));

    //  直進
    leftPWM = 15;
    rightPWM = 15;
    distance = 13.5;
    Walker *walker8 = new Walker(leftPWM, rightPWM);
    WheelDistancePredicate *walker8Predicate = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(walker8, walker8Predicate, GET_VARIABLE_NAME(walker8));

    // カーブ
#ifdef SimulatorMode
    pwm = 20;
#else
    pwm = 7;
#endif
    r = 23;
    theta = 40;
    CurvatureWalkerCommandAndPredicate *curve5 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
    commandExecutor->addCommand(curve5->getCommand(), curve5->getPredicate(), GET_VARIABLE_NAME(curve5));

    // カーブ
#ifdef SimulatorMode
    pwm = 20;
#else
    pwm = 7;
#endif
    r = 10;
    theta = -65;
    CurvatureWalkerCommandAndPredicate *curve6 = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, r, theta, robotAPI);
    commandExecutor->addCommand(curve6->getCommand(), curve6->getPredicate(), GET_VARIABLE_NAME(curve6));

    // PIDで青線まで進む
    commandExecutor->addCommand(lowPWMTracer, new ColorPredicate(COLOR_BLUE), GET_VARIABLE_NAME(lowPWMTracer));

    // 指示待ち走行ここまで
#endif

    commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif