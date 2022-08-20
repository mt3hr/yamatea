#ifndef RotateRobotUseGyroCommandAndPredicate_H
#define RotateRobotUseGyroCommandAndPredicate_H

#include "CommandAndPredicate.h"

class RotateRobotUseGyroCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    RotateRobotUseGyroCommandAndPredicate(int targetAngle, int pwm, RobotAPI *robotAPI);
    virtual ~RotateRobotUseGyroCommandAndPredicate();
};


#endif