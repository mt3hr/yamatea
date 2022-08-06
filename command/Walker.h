#ifndef Walker_H
#define Walker_H

#include "Motor.h"
#include "Command.h"
#include "PrintMessage.h"
#include "WheelController.h"

using namespace ev3api;

// Walker
// 左右モータのpwmを設定してそれに従って走行させ続けるもの
//
// 実方
class Walker : public Command
{
private:
    int leftPow;
    int rightPow;
    WheelController *wheelController;
    PrintMessage *printMessage; // NOTE モデルには反映しません

public:
    Walker(int leftPow, int rightPow, WheelController *wheelController);
    ~Walker();
    void run() override;
    Walker *generateReverseCommand() override;
};

#endif