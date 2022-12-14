#ifndef Walker_H
#define Walker_H

#include "Motor.h"
#include "Command.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

// Walker
// 左右モータのpwmを設定してそれに従って走行させるコマンド
//
// 実方
class Walker : public Command
{
private:
    float leftPower;
    float rightPower;

public:
    Walker(float leftPower, float rightPower);
    virtual ~Walker();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Walker *generateReverseCommand() override;
};

#endif