#ifndef PIDTargetColorBrightnessCalibrator_H
#define PIDTargetColorBrightnessCalibrator_H

#include <vector>
#include "ColorSensor.h"
#include "Command.h"
#include "Clock.h"
#include "string"
#include "RobotAPI.h"
#include "PIDTracer.h"
#include "ColorPIDTracer.h"
#include "Sensor.h"

using namespace ev3api;
using namespace std;

class PIDTargetColorBrightnessCalibrator : public Command
{
private:
    bool printedReadBlackMessage = false; // NOTE モデルに反映しません。
    bool printedReadWhiteMessage = false; // NOTE モデルに反映しません。
    bool printedReadBlackColorMessage = false; // NOTE モデルに反映しません。
    bool printedReadWhiteColorMessage = false; // NOTE モデルに反映しません。
    int whiteBrightness = 0;
    int blackBrightness = 100;
    rgb_raw_t whiteColor;
    rgb_raw_t blackColor;
    bool readedWhiteBrightness = false;
    bool readedBlackBrightness = false;
    bool readedWhiteColor = false;
    bool readedBlackColor = false;
    vector<PIDTracer *> pidTracers;
    vector<ColorPIDTracer *> colorPIDTracers;
    bool calibratedPIDTracers = false;
    RobotAPI *robotAPI;

public:
    PIDTargetColorBrightnessCalibrator(RobotAPI *robotAPI);
    virtual ~PIDTargetColorBrightnessCalibrator();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDTargetColorBrightnessCalibrator *generateReverseCommand() override;
    virtual int getBlackBrightness();
    virtual int getWhiteBrightness();
    virtual int getBlackColorRedValue();
    virtual int getWhiteColorRedValue();
    virtual bool isReadedBlackBrightness();
    virtual bool isReadedWhiteBrightness();
    virtual bool isReadedBlackColorRedValue();
    virtual bool isReadedWhiteColorRedValue();
    virtual void readWhiteBrightnessFromColorSensor();
    virtual void readBlackBrightnessFromColorSensor();
    virtual void readWhiteColorRedValueFromColorSensor();
    virtual void readBlackColorRedValueFromColorSensor();
    virtual void addPIDTracer(PIDTracer *pidTracer);
    virtual void addColorPIDTracer(ColorPIDTracer *pidTracer);
};

#endif
