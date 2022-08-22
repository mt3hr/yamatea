#ifndef ArmController_H
#define ArmController_H

#include "Motor.h"
#include "Command.h"
#include "RobotAPI.h"

using namespace ev3api;

// ArmController
// カラーセンサの乗ったアームを動かすコマンド。
// +で上げ、-で下げる。
//
// 実方
class ArmController : public Command
{
private:
    int pwm;

public:
    ArmController(int pwm);
    virtual ~ArmController();
    virtual void run(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ArmController *generateReverseCommand() override;
};

#endif