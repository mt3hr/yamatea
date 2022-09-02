#ifndef TailController_H
#define TailController_H

#include "Motor.h"
#include "Command.h"
#include "RobotAPI.h"

using namespace ev3api;

// TailController
// テールモータを動かすコマンド。
// +で上げ、-で下げる。
//
// 実方
class TailController : public Command
{
private:
    int pwm;

public:
    TailController(int pwm);
    virtual ~TailController();
    virtual void run(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual TailController *generateReverseCommand() override;
};

#endif