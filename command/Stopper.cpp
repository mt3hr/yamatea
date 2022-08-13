#include "Stopper.h"
#include "Setting.h"
#include "DebugUtil.h"
#include "RobotAPI.h"

using namespace ev3api;

Stopper::Stopper()
{
}

void Stopper::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->stop();
    robotAPI->getRightWheel()->stop();

    writeDebug("Stopper");
    writeEndLineDebug();
    writeDebug("stopped.");
    flushDebug(INFO, robotAPI);
}

Stopper *Stopper::generateReverseCommand()
{
    return new Stopper();
}