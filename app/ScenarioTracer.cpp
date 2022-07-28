#include "ScenarioTracer.h"

ScenarioTracer::ScenarioTracer(int lp, int rp, Motor *lw, Motor *rw)
{
    leftPow = lp;
    rightPow = rp;
    leftWheel = lw;
    rightWheel = rw;
}

void ScenarioTracer::run()
{
    leftWheel->setPWM(leftPow);
    rightWheel->setPWM(rightPow);
}

ScenarioTracer *ScenarioTracer::generateReverseCommand()
{
    return new ScenarioTracer(rightPow, leftPow, leftWheel, rightWheel);
}