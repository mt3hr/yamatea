#include "ScenarioTracer.h"
#include "WheelController.h"
#include "util.h"

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

    char lStr[30];
    char rStr[30];
    sprintf(lStr, "leftPow :%d", leftPow);
    sprintf(rStr, "rightPow:%d", rightPow);
    msg_f("scenario tracing", 1);
    msg_f(lStr, 2);
    msg_f(rStr, 3);
    msg_f("", 4);
    msg_f("", 5);
    msg_f("", 6);
    msg_f("", 7);
}

ScenarioTracer *ScenarioTracer::generateReverseCommand()
{
    return new ScenarioTracer(rightPow, leftPow, wheelController);
}