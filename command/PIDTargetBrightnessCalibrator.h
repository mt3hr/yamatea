#ifndef ColorSensor_H
#define ColorSensor_H

#include <vector>
#include "ColorSensor.h"
#include "Command.h"
#include "Handler.h"
#include "Clock.h"
#include "string"

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
    bool printedReadBlackMessage = false;  // NOTE モデルに反映しません。
    bool printedReadWhiteMessage = false;  // NOTE モデルに反映しません。
    bool printedCalibratedMessage = false; // NOTE モデルに反映しません。
    int white = 0;
    int black = 100;
    bool readedWhite = false;
    bool readedBlack = false;
    ColorSensor *colorSensor;
    vector<Handler *> handlers;
    bool executedHandler = false;
    Clock *clock;

public:
    PIDTargetBrightnessCalibrator(ColorSensor *colorSensor, Clock *clock);
    void run() override;
    PIDTargetBrightnessCalibrator *generateReverseCommand() override;
    int getBlack();
    int getWhite();
    bool isReadedBlack();
    bool isReadedWhite();
    void readWhiteFromColorSensor();
    void readBlackFromColorSensor();
    void addRoadedHandler(Handler *handler);
};

#endif
