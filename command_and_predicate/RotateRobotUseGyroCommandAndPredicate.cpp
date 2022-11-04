#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "Walker.h"
#include "GyroRotateAnglePredicate.h"

RotateRobotUseGyroCommandAndPredicate::RotateRobotUseGyroCommandAndPredicate(int targetAngle, float pwm, RobotAPI *robotAPI)
{
    Command *command;
    if (targetAngle >= 0)
    {
        command = new Walker(pwm, -pwm);
    }
    else
    {
        command = new Walker(-pwm, pwm);
    }

    GyroRotateAnglePredicate *predicate = new GyroRotateAnglePredicate(targetAngle);

    setCommand(command);
    setPredicate(predicate);
}

RotateRobotUseGyroCommandAndPredicate::~RotateRobotUseGyroCommandAndPredicate()
{
}
