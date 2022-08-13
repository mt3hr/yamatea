#include "SetPIDTargetBrightnessWhenCalibratedHandler.h"
#include "Handler.h"
#include "PIDTargetBrightnessCalibrator.h"
#include "PIDTracer.h"
#include "RobotAPI.h"

SetPIDTargetBrightnessWhenCalibratedHandler::SetPIDTargetBrightnessWhenCalibratedHandler(PIDTracer *pt, PIDTargetBrightnessCalibrator *ptbc)
{
    pidTracer = pt;
    pidTargetBrightnessCalibrator = ptbc;
}

void SetPIDTargetBrightnessWhenCalibratedHandler::handle(RobotAPI *robotAPI)
{
    pidTracer->setTargetBrightness(calcTargetBrightness());
}

int SetPIDTargetBrightnessWhenCalibratedHandler::calcTargetBrightness()
{
    return (pidTargetBrightnessCalibrator->getBlack() + pidTargetBrightnessCalibrator->getWhite()) / 2;
}
