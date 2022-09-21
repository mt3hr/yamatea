#ifndef RotateRobotCommandAndPredicateV2_H
#define RotateRobotCommandAndPredicateV2_H

#include "CommandAndPredicate.h"

// RotateRobotCommandAndPredicateV2
// ロボットを指定角度旋回させるためのCommandとPredicate。
// 精度が求められる場合はcommand/FacingAngleを使って。
// 
// 実方
class RotateRobotCommandAndPredicateV2 : public CommandAndPredicate
{
private:
public:
    RotateRobotCommandAndPredicateV2(int targetAngle, int pwm, RobotAPI *robotAPI);
    virtual ~RotateRobotCommandAndPredicateV2();
};

#endif