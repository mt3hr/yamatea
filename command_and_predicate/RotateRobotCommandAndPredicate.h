#ifndef RotateRobotCommandAndPredicate_H
#define RotateRobotCommandAndPredicate_H

#include "CommandAndPredicate.h"

class RotateRobotCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    RotateRobotCommandAndPredicate(int targetAngle, int pwm, RobotAPI *robotAPI);
};

#endif