#include "Walker.h"
#include "WheelController.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "DebugUtil.h"

using namespace std;

Walker::Walker(int lp, int rp, WheelController *wc)
{
    leftPower = lp;
    rightPower = rp;
    wheelController = wc;
}

void Walker::run()
{
    wheelController->getLeftWheel()->setPWM(leftPower);
    wheelController->getRightWheel()->setPWM(rightPower);

    writeDebug("Walker");
    writeEndLineDebug();
    writeDebug("leftPow: ");
    writeDebug(leftPower);
    writeEndLineDebug();
    writeDebug("rightPow: ");
    writeDebug(rightPower);
    flushDebug();
}

Walker *Walker::generateReverseCommand()
{
    return new Walker(rightPower, leftPower, wheelController);
}