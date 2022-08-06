#include "Walker.h"
#include "WheelController.h"
#include "PrintMessage.h"
#include "Setting.h"
#include "string"
#include "sstream"
#include "vector"
#include "iomanip"

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

    // メッセージ出力処理。変数名は雑。
    if (enablePrintMessageMode)
    {
        stringstream ls;
        stringstream rs;

        ls.clear();
        rs.clear();

        ls.str("");
        rs.str("");

        ls << "leftPow :" << float(leftPower);  // intのままだと出力されないのでfloatに変換する
        rs << "rightPow:" << float(rightPower); // intのままだと出力されないのでfloatに変換する

        vector<string> messageLines;
        messageLines.push_back("walking");
        messageLines.push_back(ls.str());
        messageLines.push_back(rs.str());
        PrintMessage printMessage(messageLines, false);
        printMessage.run();
    }
}

Walker *Walker::generateReverseCommand()
{
    return new Walker(rightPower, leftPower, wheelController);
}