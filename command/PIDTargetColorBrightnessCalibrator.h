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

enum BrightnessCalibrateMode
{
    BCM_BlackWhiteAverage,
    BCM_BlackWhiteEdge,
};

class PIDTargetColorBrightnessCalibrator : public Command
{
private:
    BrightnessCalibrateMode brightnessCalibrateMode;
    bool printedReadBlackMessage = false;            // NOTE モデルに反映しません。
    bool printedReadWhiteMessage = false;            // NOTE モデルに反映しません。
    bool printedReadRedMessage = false;              // NOTE モデルに反映しません。
    bool printedReadGreenMessage = false;            // NOTE モデルに反映しません。
    bool printedReadBlueMessage = false;             // NOTE モデルに反映しません。
    bool printedReadYellowMessage = false;           // NOTE モデルに反映しません。
    bool printedReadBlackColorMessage = false;       // NOTE モデルに反映しません。
    bool printedReadWhiteColorMessage = false;       // NOTE モデルに反映しません。
    bool printedReadBlueEdgeMessage = false;         // NOTE モデルに反映しません。
    bool printedReadSlalomWhiteColorMessage = false; // NOTE モデルに反映しません。
    bool printedReadBlackWhiteEdgeMessage = false;
    bool printedReadGrayColorMessage = false;
    bool printedResetAPIMessage = false; // NOTE モデルに反映しません。
    bool readedBlackWhiteEdgeBrightness = false;
    bool readedBlackWhiteEdge = false;
    bool readedWhiteBrightness = false;
    bool readedBlackBrightness = false;
    bool readedWhiteColor = false;
    bool readedBlackColor = false;
    bool readedRedColor = false;
    bool readedGreenColor = false;
    bool readedBlueColor = false;
    bool readedYellowColor = false;
    bool readedBlueEdgeColor = false;
    bool readedSlalomWhiteColor = false;
    bool readedBlackWhiteEdgeColor = false;
    bool readedGrayColor = false;
    bool resetedAPI = false;
    bool wrotedToFile = false;
    vector<PIDTracer *> pidTracers;
    vector<ColorPIDTracer *> colorPIDTracers;
    bool calibratedPIDTracers = false;
    RobotAPI *robotAPI;

public:
    PIDTargetColorBrightnessCalibrator(RobotAPI *robotAPI, BrightnessCalibrateMode mode);
    virtual ~PIDTargetColorBrightnessCalibrator();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDTargetColorBrightnessCalibrator *generateReverseCommand() override;
    virtual bool isReadedBlackBrightness();
    virtual bool isReadedWhiteBrightness();
    virtual bool isReadedBlackColor();
    virtual bool isReadedWhiteColor();
    virtual bool isReadedBlackWhiteEdgeBrightness();
    virtual bool isReadedBlackWhiteEdgeColor();
    virtual bool isResetedAPI();
    virtual void readWhiteBrightnessFromColorSensor();
    virtual void readBlackBrightnessFromColorSensor();
    virtual void readWhiteColorFromColorSensor();
    virtual void readBlackColorFromColorSensor();
    virtual void readBlackWhiteEdgeColorFromColorSensor();
    virtual void readBlackWhiteEdgeBrightnessFromColorSensor();
    virtual void resetAPI();
    virtual void addPIDTracer(PIDTracer *pidTracer);
    virtual void addColorPIDTracer(ColorPIDTracer *pidTracer);
    virtual vector<PIDTracer *> *getPIDTracers();
    virtual vector<ColorPIDTracer *> *getColorPIDTracers();
};

#endif
