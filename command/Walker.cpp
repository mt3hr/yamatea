#include "Walker.h"
#include "PrintMessage.h"
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

    if (printMessage)
    {
        char lStr[30];
        char rStr[30];
        sprintf(lStr, "leftPow :%d\r\n", leftPow);
        sprintf(rStr, "rightPow:%d\r\n", rightPow);
        msg_f("scenario tracing\r\n", 1);
        msg_f(lStr, 2);
        msg_f(rStr, 3);
        msg_f("", 4);
        msg_f("", 5);
        msg_f("", 6);
        msg_f("", 7);
    }
}

Walker *Walker::generateReverseCommand()
{
    return new Walker(rightPow, leftPow, wheelController);
}