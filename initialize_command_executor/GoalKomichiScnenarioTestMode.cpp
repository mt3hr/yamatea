#include "Setting.h"
#ifdef GoalKomichiScnenarioTestMode

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
    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;

    float leftPow;
    float rightPow;
    int distance;

    PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
    Predicate *startButtonPredicate = new StartButtonPredicate();
    commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

#define SonarStarter
#ifdef SonarStarter
    Stopper *sonarStandby = new Stopper();
    Predicate *sonarStater = new SonarDistancePredicate(10, true);
    commandExecutor->addCommand(sonarStandby, sonarStater, GET_VARIABLE_NAME(sonarStandby));
#endif

    pwm = 22;
    kp = 0.7;
    ki = 0.2;
    kd = 0.7;
    dt = 1;
    distance = 8;
    PIDTracer *bananaPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt);
    calibrator->addPIDTracer(bananaPIDTracer);

    leftPow = 0;
    rightPow = 0;
    Walker *walkerS = new Walker(leftPow, rightPow);
    // Predicate *predicate0 = new NumberOfTimesPredicate(4);
    Predicate *predicate0 = new WheelDistancePredicate(distance, robotAPI);
    commandExecutor->addCommand(bananaPIDTracer, predicate0, GET_VARIABLE_NAME(bananaPIDTracer));
    // 第一直進
    leftPow = 50;
    rightPow = 50;
    walkerS = new Walker(leftPow, rightPow);
    // Predicate *predicate1 = new WheelDistancePredicate(40, robotAPI);
    Predicate *predicate1 = new WheelDistancePredicate(20, robotAPI);
    commandExecutor->addCommand(walkerS, predicate1, GET_VARIABLE_NAME(walkerS));
    // 第二カーブ
    leftPow = 50;
    rightPow = 10;
    Walker *walker2 = new Walker(leftPow, rightPow);
    Predicate *predicate2 = new WheelDistancePredicate(24, robotAPI);
    commandExecutor->addCommand(walker2, predicate2, GET_VARIABLE_NAME(walker2));
    // 第三直進
    leftPow = 50;
    rightPow = 50;
    // Walker *walker3 = new Walker(leftPow, rightPow);
    Predicate *predicate3 = new WheelDistancePredicate(52, robotAPI);
    commandExecutor->addCommand(walkerS, predicate3, GET_VARIABLE_NAME(walkerS));
    // 第四カーブ
    leftPow = 50;
    rightPow = 10;
    Walker *walker4 = new Walker(leftPow, rightPow);
    Predicate *predicate4 = new WheelDistancePredicate(25, robotAPI);
    commandExecutor->addCommand(walker4, predicate4, GET_VARIABLE_NAME(walker4));
    // 5直進
    Predicate *predicate5 = new WheelDistancePredicate(10, robotAPI);
    commandExecutor->addCommand(walkerS, predicate5, GET_VARIABLE_NAME(walkerS));
    // 第6カーブ,mid1
    leftPow = 30;
    rightPow = 50;
    Walker *walker6 = new Walker(leftPow, rightPow);
    Predicate *predicate6 = new WheelDistancePredicate(70, robotAPI);
    commandExecutor->addCommand(walker6, predicate6, GET_VARIABLE_NAME(walker6));
    Predicate *predicateMid1 = new WheelDistancePredicate(9, robotAPI);
    commandExecutor->addCommand(walkerS, predicateMid1, GET_VARIABLE_NAME(walkerS));
    // OK第7　一度目交差点から丸一周
    leftPow = 50;
    rightPow = 36;
    Walker *walker7 = new Walker(leftPow, rightPow);
    Predicate *predicate7 = new WheelDistancePredicate(340, robotAPI);
    commandExecutor->addCommand(walker7, predicate7, GET_VARIABLE_NAME(walker7));
    // 第8　2度目交差点から抜ける
    leftPow = 20;
    rightPow = 50;
    Walker *walker8 = new Walker(leftPow, rightPow);
    Predicate *predicate8 = new WheelDistancePredicate(8, robotAPI);
    commandExecutor->addCommand(walker8, predicate8, GET_VARIABLE_NAME(walker8));
    // 9直進
    Predicate *predicate9 = new WheelDistancePredicate(42, robotAPI);
    commandExecutor->addCommand(walkerS, predicate9, GET_VARIABLE_NAME(walkerS));
    // 10カーブ
    leftPow = 8;
    rightPow = 50;
    // Walker *walker10 = new Walker(leftPow, rightPow);
    Predicate *predicate10 = new WheelDistancePredicate(5, robotAPI);
    commandExecutor->addCommand(walker8, predicate10, GET_VARIABLE_NAME(walker8));
    // 11直進
    Predicate *predicate11 = new WheelDistancePredicate(65, robotAPI);
    commandExecutor->addCommand(walkerS, predicate11, GET_VARIABLE_NAME(walkerS));
    // 12Uカーブ
    leftPow = 50;
    rightPow = 12;
    Walker *walker12 = new Walker(leftPow, rightPow);
    Predicate *predicate12 = new WheelDistancePredicate(26, robotAPI);
    commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
    predicate12 = new WheelDistancePredicate(25, robotAPI);
    Predicate *predicateS12 = new WheelDistancePredicate(27, robotAPI);
    commandExecutor->addCommand(walkerS, predicateS12, GET_VARIABLE_NAME(walkerS));
    commandExecutor->addCommand(walker12, predicate12, GET_VARIABLE_NAME(walker12));
    commandExecutor->addCommand(walkerS, predicate11, GET_VARIABLE_NAME(walkerS));
    // IF１３コースに沿って直進
    Predicate *predicate13S = new WheelDistancePredicate(155, robotAPI);
    commandExecutor->addCommand(walkerS, predicate13S, GET_VARIABLE_NAME(walkerS));
    leftPow = 13;
    rightPow = 50;
    Walker *walker13 = new Walker(leftPow, rightPow);
    Predicate *predicate13 = new WheelDistancePredicate(7, robotAPI);
    commandExecutor->addCommand(walker13, predicate13, GET_VARIABLE_NAME(walker13));
    Predicate *predicate13S2 = new WheelDistancePredicate(20, robotAPI);
    commandExecutor->addCommand(walkerS, predicate13S2, GET_VARIABLE_NAME(walkerS));
    //   //13ゴールに向かって直進
    //  leftPow = 31;
    // rightPow = 100;
    // Walker *walker13 = new Walker(leftPow, rightPow);
    // Predicate *predicate13 = new WheelDistancePredicate(2, robotAPI);
    // commandExecutor->addCommand(walker13, predicate13, GET_VARIABLE_NAME(walker13));
    // Predicate *predicate13S = new WheelDistancePredicate(151, robotAPI);
    // commandExecutor->addCommand(walkerS, predicate13S, GET_VARIABLE_NAME(walkerS));
    // Predicate *predicate13t = new WheelDistancePredicate(36, robotAPI);
    // commandExecutor->addCommand(walker13, predicate13t, GET_VARIABLE_NAME(walker13));
    // 14黒キャッチしてから　ライントレース
    /*
    leftPow = 20;
    rightPow = 20;
    Walker *walker14S = new Walker(leftPow, rightPow);
    Predicate *predicate14c = new ColorPredicate(COLOR_BLACK);
    commandExecutor->addCommand(walker14S, predicate14c, GET_VARIABLE_NAME(walker14S));
    */

    pwm = 10;
    float angle = -45;
    FacingAngleAbs *facingAngle45 = new FacingAngleAbs(FA_Gyro, pwm, angle);
    commandExecutor->addCommand(facingAngle45, new FinishedCommandPredicate(facingAngle45), GET_VARIABLE_NAME(facingAngle45));

    leftPow = 20;
    rightPow = 20;
    Walker *lowWalker = new Walker(leftPow, rightPow);
    Predicate *blackPredicate = new BlackPredicate();
    commandExecutor->addCommand(lowWalker, blackPredicate, GET_VARIABLE_NAME(lowWalker));

    pwm = 15;
    kp = 0.2;
    ki = 0.1;
    kd = 0.2;
    dt = 1;
    ColorPIDTracer *colorPIDTracer = new ColorPIDTracer(RIGHT_TRACE, Trace_R, pwm, kp, ki, kd, dt);
    calibrator->addColorPIDTracer(colorPIDTracer);
    commandExecutor->addCommand(colorPIDTracer, new BlueEdgePredicate(), GET_VARIABLE_NAME(colorPIDTracer));
}
#endif