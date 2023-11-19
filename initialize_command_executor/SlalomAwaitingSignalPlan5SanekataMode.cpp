#include "Setting.h"
#ifdef SlalomAwaitingSignalPlan5SanekataMode

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
        // colorid_t *garageCardColorPtr = colorReader->getColorPtr();

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
                        theta = 50;
                        CurvatureWalkerCommandAndPredicate *curveA = new CurvatureWalkerCommandAndPredicate(CWCMP_WheelCount, pwm, radius, theta, robotAPI);
                        commandExecutor->addCommand(curveA->getCommand(), curveA->getPredicate(), GET_VARIABLE_NAME(curveA));
                        commandExecutor->addCommand(stopper, new NumberOfTimesPredicate(1), GET_VARIABLE_NAME(stopper));

                        // カーブ
                        pwm = 5 * coefficientPWMForCurve;
                        radius = 10.8; // 11.5;
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
                        theta = -87.5;
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
}
#endif