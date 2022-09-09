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
    bool printedReadBlackMessage = false;            // NOTE モデルに反映しません。
    bool printedReadWhiteMessage = false;            // NOTE モデルに反映しません。
    bool printedReadBlueMessage = false;             // NOTE モデルに反映しません。
    bool printedReadBlackColorMessage = false;       // NOTE モデルに反映しません。
    bool printedReadWhiteColorMessage = false;       // NOTE モデルに反映しません。
    bool printedReadBlueEdgeMessage = false;         // NOTE モデルに反映しません。
    bool printedReadSlalomWhiteColorMessage = false; // NOTE モデルに反映しません。
    bool printedReadBlackWhiteEdgeMessage = false;
    bool printedResetAPIMessage = false; // NOTE モデルに反映しません。
    int whiteBrightness;
    int blackBrightness;
    int blackWhiteEdgeBrightness;
    rgb_raw_t whiteColor;
    rgb_raw_t blackColor;
    rgb_raw_t blackWhiteEdgeColor;
    bool readedBlackWhiteEdgeBrightness = false;
    bool readedBlackWhiteEdge = false;
    bool readedWhiteBrightness = false;
    bool readedBlackBrightness = false;
    bool readedWhiteColor = false;
    bool readedBlackColor = false;
    bool readedBlueColor = false;
    bool readedBlueEdgeColor = false;
    bool readedSlalomWhiteColor = false;
    bool readedBlackWhiteEdgeColor = false;
    bool resetedAPI = false;
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
    virtual rgb_raw_t getBlackColor();
    virtual rgb_raw_t getWhiteColor();
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
};

#endif
