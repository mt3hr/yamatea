#include "Walker.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "DebugUtil.h"
#include "RobotAPI.h"

using namespace std;

Walker::Walker(float lp, float rp)
{
    leftPower = lp;
    rightPower = rp;
}

Walker::~Walker()
{
}

void Walker::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->setPWM(leftPower);
    robotAPI->getRightWheel()->setPWM(rightPower);
}

void Walker::preparation(RobotAPI *robotAPI)
{
    writeDebug("Walker");
    writeEndLineDebug();
    writeDebug("leftPow: ");
    writeDebug(leftPower);
    writeEndLineDebug();
    writeDebug("rightPow: ");
    writeDebug(rightPower);
    flushDebug(DEBUG, robotAPI);
    return;
}

Walker *Walker::generateReverseCommand()
{
    return new Walker(rightPower, leftPower);
}