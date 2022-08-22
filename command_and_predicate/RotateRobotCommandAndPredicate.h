#ifndef RotateRobotCommandAndPredicate_H
#define RotateRobotCommandAndPredicate_H

#include "CommandAndPredicate.h"

// 非推奨。RotateRobotUseGyroCommandAndPredicateのほうが高精度（このクラスは車輪回転数から逆算しているが、あまり精度が良くない）
// RotateRobotCommandAndPredicate
// ロボットを指定角度旋回させるためのCommandとPredicate。
// 
// 実方
class RotateRobotCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    RotateRobotCommandAndPredicate(int targetAngle, int pwm, RobotAPI *robotAPI);
    virtual ~RotateRobotCommandAndPredicate();
};

#endif