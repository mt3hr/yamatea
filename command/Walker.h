#ifndef Walker_H
#define Walker_H

#include "Motor.h"
#include "Command.h"
#include "WheelController.h"

using namespace ev3api;
using namespace std;

// Walker
// 左右モータのpwmを設定してそれに従って走行させ続けるもの
//
// 実方
class Walker : public Command
{
private:
    int leftPower;
    int rightPower;
    WheelController *wheelController;

public:
    Walker(int leftPower, int rightPower, WheelController *wheelController);
    void run() override;
    Walker *generateReverseCommand() override;
};

#endif