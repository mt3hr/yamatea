#include "Stopper.h"
#include "Setting.h"
#include "DebugUtil.h"
#include "RobotAPI.h"

using namespace ev3api;

Stopper::Stopper()
{
}

Stopper::~Stopper()
{
}

void Stopper::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->stop();
    robotAPI->getRightWheel()->stop();
    robotAPI->getArmMotor()->stop();
    robotAPI->getTailMotor()->stop();

    writeDebug("Stopper");
    writeEndLineDebug();
    writeDebug("stopped.");
    flushDebug(INFO, robotAPI);
}

void Stopper::preparation(RobotAPI *robotAPI)
{
    return;
}

Stopper *Stopper::generateReverseCommand()
{
    return new Stopper();
}