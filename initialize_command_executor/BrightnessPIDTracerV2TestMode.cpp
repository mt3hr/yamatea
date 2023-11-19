#include "Setting.h"
#ifdef BrightnessPIDTracerV2TestMode

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

  PIDTargetColorBrightnessCalibrator *calibrator = new PIDTargetColorBrightnessCalibrator(robotAPI, BCM_BlackWhiteAverage);
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(calibrator, startButtonPredicate, GET_VARIABLE_NAME(calibrator));

  float pwm;
  float kp;
  float ki;
  float kd;
  float dt;
  float r;

  // pwm60 円コース。未完成
  pwm = 60;
  kp = 0.881;
  ki = 0.025;
  kd = 0.952;
  dt = 1;
  r = 0;

  // pwm40 円コース
  pwm = 40;
  kp = 0.6265;
  ki = 0.0812;
  kd = 1.9221;
  dt = 1;
  r = 0;

  // すいか爆速R制御
  pwm = 65;
  kp = 0.8;
  ki = 0;
  kd = 4.4;
  dt = 1;
  r = 20;

  // おれんじ爆速R制御
  pwm = 65;
  kp = 0.9;
  ki = 0;
  kd = 4.4;
  dt = 1;
  r = -34;

  // きゅうり爆速
  pwm = 85;
  kp = 0.8;
  ki = 0;
  kd = 4.4;
  dt = 1;
  r = 0;

  // アスパラガス
  pwm = 30;
  kp = 0.6;
  ki = 0;
  kd = 2.0;
  dt = 1;
  r = 0;

  // すいか爆速R制御i制御あり
  pwm = 65;
  kp = 0.7;
  ki = 0.3;
  kd = 2.0;
  dt = 0.45;
  r = 20;

  // きゅうり爆速i制御あり
  pwm = 75;
  kp = 0.95;
  ki = 0.3;
  kd = 2.4;
  dt = 0.4;
  r = 0;

  // もも
  pwm = 40;
  kp = 0.25;
  ki = 0.35;
  kd = 2.7;
  dt = 0.5;
  r = 17;

  // 60ぐらいの直進PID
  pwm = 60;
  kp = 0.4;
  ki = 0.12;
  kd = 1.2;
  dt = 0.4;
  r = 0;

  // おれんじ爆速R制御i制御あり
  pwm = 60;
  kp = 0.4;
  ki = 0.12;
  kd = 1.2;
  dt = 0.4;
  r = -32.5;

  // すいか爆速R制御i制御あり
  pwm = 65;
  kp = 0.5;
  ki = 0.1;
  kd = 1.5;
  dt = 0.45;
  r = 26;

  // どりあん
  pwm = 40;
  kp = 0.7;
  ki = 0.015;
  kd = 0.43;
  dt = 0.35;
  r = 0;

  // どりあん
  pwm = 40;
  kp = 0.85;
  ki = 0;
  kd = 2.25;
  dt = 1;
  r = 0;

  // どりあん
  pwm = 35;
  kp = 0.5;
  ki = 0.01;
  kd = 1.5;
  dt = 1;
  r = 0;

  // おれんじ爆速R制御i制御あり
  pwm = 60;
  kp = 0.4;
  ki = 0.12;
  kd = 0.8;
  dt = 0.4;
  r = 0; // -28; // TODO 直進できる値を探してからRを探して

  // おれんじ爆速R制御
  pwm = 60;
  kp = 0.675;
  ki = 0;
  kd = 2.3;
  dt = 1;
  r = -26;

  // すいか爆速R制御
  pwm = 65;
  kp = 0.7;
  ki = 0;
  kd = 2;
  dt = 1;
  r = 32;

  // float carrotPWM = 60;
  float carrotKp = 0.35;
  // float carrotKi = 0.015; // 0.12;
  // float carrotKd = carrotKp * 3;
  // float carrotDt = 0.4;
  // float carrotR = 55;

  // いちご
  pwm = 60;
  kp = 0.615;
  ki = 0;  // 0.15;
  kd = kp; // TODO オーバーシュートしているのならばKdが高すぎた説がありえます
  dt = 1;
  r = -40;

  // きゅうり
  float defaultKi = 0.05;
  float magnificationKd = 0.05;
  pwm = 65;
  kp = 0.8;
  ki = defaultKi;
  kd = carrotKp * magnificationKd;
  dt = 0.01;
  r = 0;

  // 1252で求めた値
  pwm = 65;
  kp = 0.6;
  ki = 2;
  kd = 0.18;
  dt = 0.5;

  // i下げる
  pwm = 65;
  kp = 0.6;  // pもまぁまぁいい感じ
  ki = 2.65; // iはとてもいい感じ
  kd = 0.25; // dtが少なすぎてkdの調節がやりづらくなっている
  dt = 0.05; // dtが少なすぎてkdの調節がやりづらくなっている

  // 実機で明日（2022-11-11）試す値1
  dt = 0.5;
  kp = 0.6;
  ki = 1.65;
  kd = 0.25;

  // 実機で明日（2022-11-11）試す値3
  dt = 0.75;
  kp = 0.6;
  ki = 1.9;
  kd = 0.29;

  // 実機で明日（2022-11-11）試す値4
  dt = 0.5;
  kp = 0.35;
  ki = 2.2;
  kd = 0.29;

  // 実機で明日（2022-11-11）試す値2
  dt = 0.5;
  kp = 0.6;
  ki = 1.65;
  kd = 0.1338;

  // i下げる
  pwm = 65;
  kp = 0.6;
  ki = 2.65;
  kd = 0.25;
  dt = 0.05;

  // スイカトレースできるけどカクつく
  pwm = 65;
  kp = 0.8;
  ki = 0.95;
  kd = 0.015;
  dt = 0.05;

  pwm = 65;
  kp = 0.8;
  ki = 0.65;
  kd = 0.01;
  dt = 0.05;

  pwm = 65;
  kp = 0.4;
  ki = 1.3;
  kd = 0.011;
  dt = 0.05;

  // きゅうりいい感じの値
  pwm = 65;
  kp = 0.6;
  ki = 2.65;
  kd = 0.21;
  dt = 0.05;
  r = 0;

  // にんじんのいい感じの値 //TODO
  pwm = 45;
  kp = 1.34;
  ki = 0; // 0.4;
  kd = 2.5;
  dt = 1; // 0.05;
  r = 0;  // 28;

  // pwm0の限界感度調査
  pwm = 0;
  kp = 3;
  ki = 0;
  kd = 0;
  dt = 0.05;
  r = 0;

  // にんじんの限界感度調査
  pwm = 45;
  kp = 2.2;
  ki = 0;
  kd = 0;
  dt = 0.05;
  r = 0;

  // にんじんのいい感じの値沖原おれんじベース
  pwm = 45;
  kp = 1.2;
  ki = 0;
  kd = 2.0;
  dt = 1;
  r = 0;

  pwm = 45;
  kp = 1.1;
  ki = 0;
  kd = 2.2;
  dt = 1;
  r = 0;

  // オレンジいい感じの値 //TODO
  pwm = 45;
  kp = 0.6;
  ki = 2.65;
  kd = 0.21;
  dt = 0.05;
  r = 0; // -23;

  // スイカいい感じの値 //TODO
  pwm = 55;
  kp = 0.6;
  ki = 2.65;
  kd = 0.21;
  dt = 0.05;
  r = 0; // 19;

  // にんじんの限界感度調査
  pwm = 45;
  kp = 1.8;
  ki = 0;
  kd = 0;
  dt = 0.05;
  r = 0;

  // おれんじの限界感度調査
  pwm = 45;
  kp = 1.8;
  ki = 0;
  kd = 0;
  dt = 0.05;
  r = 0;

  // すいかの限界感度調査
  pwm = 55;
  kp = 4;
  ki = 0;
  kd = 0;
  dt = 0.05;
  r = 0;

  // pwm40 円コース
  pwm = 40;
  kp = 0.6265;
  ki = 0.0812;
  kd = 1.9221;
  dt = 1;
  r = 0;

  PIDTracerV2 *pidTracer = new PIDTracerV2(RIGHT_TRACE, pwm, kp, ki, kd, dt, r);
  commandExecutor->addCommand(pidTracer, new Predicate(), GET_VARIABLE_NAME(pidTracer));
  calibrator->addPIDTracer(pidTracer);
}
#endif