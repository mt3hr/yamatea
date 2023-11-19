#include "Setting.h"
#ifdef GarageKomichiMode1

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
  ColorReader *colorReader = new ColorReader();
  colorid_t *garageCardColorPtr = colorReader->getColorPtr();
  // colorid_t *garageCardColorPtr = new colorid_t(COLOR_RED);

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
}
#endif