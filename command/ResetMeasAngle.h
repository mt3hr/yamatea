#ifndef ResetMeasAngle_H
#define ResetMeasAngle_H

#include "Command.h"
#include "RobotAPI.h"

// ResetMeasAngle
// RobotAPI.MeasAngleをリセットするコマンド。
// FacingAngleを使う直前などに呼び出して。
//
// 実方
class ResetMeasAngle : public Command
{
private:
public:
    ResetMeasAngle();
    virtual ~ResetMeasAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ResetMeasAngle *generateReverseCommand() override;
};

#endif