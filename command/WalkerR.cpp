#include "WalkerR.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "DebugUtil.h"
#include "RobotAPI.h"
#include "math.h"

using namespace std;

WalkerR::WalkerR(float lp, float rp, float r)
{
    leftPower = lp;
    rightPower = rp;
    this->r = r;
}

WalkerR::~WalkerR()
{
}

void WalkerR::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->setPWM(round(leftPower + r / 2));
    robotAPI->getRightWheel()->setPWM(round(rightPower - r / 2));
}

void WalkerR::preparation(RobotAPI *robotAPI)
{
    writeDebug("WalkerR");
    writeEndLineDebug();
    writeDebug("leftPow: ");
    writeDebug(leftPower);
    writeEndLineDebug();
    writeDebug("rightPow: ");
    writeDebug(rightPower);
    flushDebug(DEBUG, robotAPI);
    return;
}

WalkerR *WalkerR::generateReverseCommand()
{
    return new WalkerR(rightPower, leftPower, -r);
}