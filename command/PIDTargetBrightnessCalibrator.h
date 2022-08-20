#ifndef ColorSensor_H
#define ColorSensor_H

#include <vector>
#include "ColorSensor.h"
#include "Command.h"
#include "Clock.h"
#include "string"
#include "RobotAPI.h"
#include "PIDTracer.h"

using namespace ev3api;
using namespace std;

// PIDTargetBrightnessCalibrator
// PIDTracerのtargetBrightnessの値を求めるために、Black、Whiteの値をセンサから取得するキャリブレータ。
// setRoadedHandler(SetPIDTargetBrightnessWhenCalibratedHandler)を使ってPIDTracerのTargetBrightnessの値を変更する。
//
// pidTargetBrightnessの値をキャリブレーション後に設定する例を下に示す。
//
// PIDTracer *commandPIDTracer = new PIDTracer(RIGHT_TRACE, pwm, kp, ki, kd, dt, targetBrightness, &leftWheel, &rightWheel, &colorSensor);
// PIDTargetBrightnessCalibrator *pidTargetBrightnessCalibrator = new PIDTargetBrightnessCalibrator(&colorSensor, &clock);
// Handler *handler = new SetPIDTargetBrightnessWhenCalibratedHandler(pidTracer, pidTargetBrightnessCalibrator);
// pidTargetBrightnessCalibrator->setRoadedHandler(handler);
//
// 実方

class PIDTargetBrightnessCalibrator : public Command
{
private:
    bool printedReadBlackMessage = false; // NOTE モデルに反映しません。
    bool printedReadWhiteMessage = false; // NOTE モデルに反映しません。
    int white = 0;
    int black = 100;
    bool readedWhite = false;
    bool readedBlack = false;
    vector<PIDTracer *> pidTracers;
    bool calibratedPIDTracers = false;
    RobotAPI *robotAPI;

public:
    PIDTargetBrightnessCalibrator(RobotAPI *robotAPI);
    void run(RobotAPI *robotAPI) override;
    PIDTargetBrightnessCalibrator *generateReverseCommand() override;
    int getBlack();
    int getWhite();
    bool isReadedBlack();
    bool isReadedWhite();
    void readWhiteFromColorSensor();
    void readBlackFromColorSensor();
    void addPIDTracer(PIDTracer *pidTracer);
};

#endif
