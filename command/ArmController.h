#ifndef ArmController_H
#define ArmController_H

#include "Motor.h"
#include "Command.h"
#include "RobotAPI.h"

using namespace ev3api;

// ArmController
// カラーセンサの乗ったアームを動かすコマンド。
//
// 実方
class ArmController : public Command
{
private:
    int pwm;

public:
    ArmController(int pwm);
    void run(RobotAPI *robotAPI);
    ArmController *generateReverseCommand() override;
};

#endif