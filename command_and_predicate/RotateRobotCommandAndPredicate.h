#ifndef RotateRobotCommandAndPredicate_H
#define RotateRobotCommandAndPredicate_H

#include "CommandAndPredicate.h"

// 非推奨。RotateRobotUseGyroCommandAndPredicateのほうが高精度
class RotateRobotCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    RotateRobotCommandAndPredicate(int targetAngle, int pwm, RobotAPI *robotAPI);
};

#endif