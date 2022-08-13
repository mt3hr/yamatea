#include "Walker.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "DebugUtil.h"
#include "RobotAPI.h"

using namespace std;

Walker::Walker(int lp, int rp)
{
    leftPower = lp;
    rightPower = rp;
}

void Walker::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->setPWM(leftPower);
    robotAPI->getRightWheel()->setPWM(rightPower);

    writeDebug("Walker");
    writeEndLineDebug();
    writeDebug("leftPow: ");
    writeDebug(leftPower);
    writeEndLineDebug();
    writeDebug("rightPow: ");
    writeDebug(rightPower);
    flushDebug(TRACE, robotAPI);
}

Walker *Walker::generateReverseCommand()
{
    return new Walker(rightPower, leftPower);
}