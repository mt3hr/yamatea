#ifndef RotateRobotUseGyroCommandAndPredicate_H
#define RotateRobotUseGyroCommandAndPredicate_H

#include "CommandAndPredicate.h"

// RotateRobotUseGyroCommandAndPredicate
// ロボットを指定角度旋回させるためのCommandとPredicate。
// 精度が求められる場合はcommand/FacingAngleを使って。
//
// 実方
class RotateRobotUseGyroCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    RotateRobotUseGyroCommandAndPredicate(int targetAngle, int pwm, RobotAPI *robotAPI);
    virtual ~RotateRobotUseGyroCommandAndPredicate();
};

#endif