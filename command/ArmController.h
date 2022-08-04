#ifndef ArmController_H
#define ArmController_H

#include "Motor.h"
#include "Command.h"

using namespace ev3api;

// ArmController
// カラーセンサの乗ったアームを動かすコマンド。
//
// 実方
class ArmController : public Command
{
private:
    int pwm;
    Motor *armMotor;

public:
    ArmController(int pwm, Motor *armMotor);
    void run();
    ArmController *generateReverseCommand() override;
};

#endif