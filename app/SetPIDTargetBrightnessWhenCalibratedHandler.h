#ifndef SetPIDTargetWhenCalibratedHandler_H
#define SetPIDTargetWhenCalibratedHandler_H

#include "Handler.h"
#include "PIDTargetBrightnessCalibrator.h"
#include "PIDTracer.h"

// SetPIDTargetBrightnessWhenCalibratedHandler
// キャリブレーションが完了したときにPIDTracerのTargetBrightnessの値を設定するHandler
//
// 実方
class SetPIDTargetBrightnessWhenCalibratedHandler : public Handler
{
private:
    PIDTracer *pidTracer;
    PIDTargetBrightnessCalibrator *pidTargetBrightnessCalibrator;
    int calcTargetBrightness();

public:
    SetPIDTargetBrightnessWhenCalibratedHandler(PIDTracer *pidTracer, PIDTargetBrightnessCalibrator *pidTargetBrightnessCalibrator);
    void handle() override;
};

#endif