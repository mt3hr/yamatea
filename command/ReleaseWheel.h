#ifndef ReleaseWheel_H
#define ReleaseWheel_H

#include "Command.h"
#include "RobotAPI.h"

// ReleaseWheel
// 左右車輪のPWMを0に設定するコマンド
//
// 実方
class ReleaseWheel : public Command
{
private:
public:
    ReleaseWheel();
    virtual ~ReleaseWheel();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ReleaseWheel *generateReverseCommand() override;
};

#endif