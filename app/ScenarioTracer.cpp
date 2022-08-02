#include "ScenarioTracer.h"
#include "WheelController.h"

ScenarioTracer::ScenarioTracer(int lp, int rp, WheelController *wc)
{
    leftPow = lp;
    rightPow = rp;
    wheelController = wc;
}

void ScenarioTracer::run()
{
    wheelController->getLeftWheel()->setPWM(leftPow);
    wheelController->getRightWheel()->setPWM(rightPow);
}

ScenarioTracer *ScenarioTracer::generateReverseCommand()
{
    return new ScenarioTracer(rightPow, leftPow, wheelController);
}