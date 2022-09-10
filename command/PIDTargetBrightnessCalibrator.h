#ifndef PIDTargetBrightnessCalibrator_H
#define PIDTargetBrightnessCalibrator_H

#include <vector>
#include "ColorSensor.h"
#include "Command.h"
#include "Clock.h"
#include "string"
#include "RobotAPI.h"
#include "PIDTracer.h"

using namespace ev3api;
using namespace std;

// 非推奨。PIDTargetColorBrightnessCalibratorを使ってください
// PIDTargetBrightnessCalibrator
// PIDTracerのtargetBrightnessの値を求めるために、Black、Whiteの値をセンサから取得するキャリブレータ。
// pidTargetBrightnessCalibrator.addPIDTracer(pidTracer);すると、キャリブレーションした値がPIDトレーサに適用されます。
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
    virtual ~PIDTargetBrightnessCalibrator();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDTargetBrightnessCalibrator *generateReverseCommand() override;
    virtual int getBlack();
    virtual int getWhite();
    virtual bool isReadedBlack();
    virtual bool isReadedWhite();
    virtual void readWhiteFromColorSensor();
    virtual void readBlackFromColorSensor();
    virtual void addPIDTracer(PIDTracer *pidTracer);
};

#endif
