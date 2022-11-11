#ifndef WalkerR_H
#define WalkerR_H

#include "Motor.h"
#include "Command.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

// WalkerR
// 左右モータのpwmを設定してそれに従って走行させるコマンド
// R補正値を探すために作りました
//
// 実方
class WalkerR : public Command
{
private:
    float leftPower;
    float rightPower;
    float r;

public:
    WalkerR(float leftPower, float rightPower, float r);
    virtual ~WalkerR();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual WalkerR *generateReverseCommand() override;
};

#endif