#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "Walker.h"
#include "GyroRotateAnglePredicate.h"

RotateRobotUseGyroCommandAndPredicate::RotateRobotUseGyroCommandAndPredicate(int targetAngle, int pwm, RobotAPI *robotAPI)
{
    bool decrease = targetAngle < 0;

    Command *command;
    if (targetAngle >= 0)
    {
        command = new Walker(-pwm, pwm);
    }
    else
    {
        command = new Walker(pwm, -pwm);
    }

    GyroRotateAnglePredicate *predicate = new GyroRotateAnglePredicate(targetAngle, decrease);

    setCommand(command);
    setPredicate(predicate);
}
