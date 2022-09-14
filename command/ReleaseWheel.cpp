#include "ReleaseWheel.h"
#include "Command.h"
#include "RobotAPI.h"

ReleaseWheel::ReleaseWheel(){};

ReleaseWheel::~ReleaseWheel(){};

void ReleaseWheel::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->setPWM(0);
    robotAPI->getRightWheel()->setPWM(0);
}

void ReleaseWheel::preparation(RobotAPI *robotAPI)
{
}

ReleaseWheel *ReleaseWheel::generateReverseCommand()
{
    return new ReleaseWheel();
}