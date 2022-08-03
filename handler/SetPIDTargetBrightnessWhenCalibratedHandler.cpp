#include "SetPIDTargetBrightnessWhenCalibratedHandler.h"
#include "Handler.h"
#include "PIDTargetBrightnessCalibrator.h"
#include "PIDTracer.h"

SetPIDTargetBrightnessWhenCalibratedHandler::SetPIDTargetBrightnessWhenCalibratedHandler(PIDTracer *pt, PIDTargetBrightnessCalibrator *ptbc)
{
    pidTracer = pt;
    pidTargetBrightnessCalibrator = ptbc;
}

void SetPIDTargetBrightnessWhenCalibratedHandler::handle()
{
    pidTracer->setTargetBrightness(calcTargetBrightness());
}

int SetPIDTargetBrightnessWhenCalibratedHandler::calcTargetBrightness()
{
    return (pidTargetBrightnessCalibrator->getBlack() + pidTargetBrightnessCalibrator->getWhite()) / 2;
}
