#include "Walker.h"
#include "WheelController.h"
#include "util.h"

Walker::Walker(int lp, int rp, WheelController *wc)
{
    leftPow = lp;
    rightPow = rp;
    wheelController = wc;
}

void Walker::run()
{
    wheelController->getLeftWheel()->setPWM(leftPow);
    wheelController->getRightWheel()->setPWM(rightPow);

#ifdef PrintMessage
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
#endif
}

Walker *Walker::generateReverseCommand()
{
    return new Walker(rightPow, leftPow, wheelController);
}