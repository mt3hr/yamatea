#include "Walker.h"
#include "WheelController.h"
#include "string"

using namespace std;

Walker::~Walker()
{
    delete printMessage;
}

Walker::Walker(int lp, int rp, WheelController *wc)
{
    leftPow = lp;
    rightPow = rp;
    wheelController = wc;

    string messageLines[] = {"walker started"};
    printMessage = new PrintMessage(messageLines);
}

void Walker::run()
{
    wheelController->getLeftWheel()->setPWM(leftPow);
    wheelController->getRightWheel()->setPWM(rightPow);

    // メッセージ出力処理
    char lStr[30];
    char rStr[30];
    sprintf(lStr, "leftPow :%d\r\n", leftPow);
    sprintf(rStr, "rightPow:%d\r\n", rightPow);

    string messageLines[] = {
        "scenario tracing\r\n",
        string(lStr),
        string(rStr),
    };
    printMessage->setMessageLines(messageLines);
    printMessage->run();
}

Walker *Walker::generateReverseCommand()
{
    return new Walker(rightPow, leftPow, wheelController);
}