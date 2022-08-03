#ifndef Walker_H
#define Walker_H

#include "Motor.h"
#include "Command.h"
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

public:
    Walker(int leftPow, int rightPow, WheelController *wheelController);
    void run() override;
    Walker *generateReverseCommand() override;
};

#endif