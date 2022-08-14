#include "Command.h"
#include "Predicate.h"
#include "CommandAndPredicate.h"
#include "Walker.h"
#include "MotorRotateAnglePredicate.h"
#include "Setting.h"

CommandAndPredicate *generateRotateRobotCommand(int targetAngle, int pwm, RobotAPI *robotAPI)
{
    int angle;
    Command *command;
    MotorRotateAnglePredicate *predicate;
    if (targetAngle > 0)
    {
        angle = ((int)(((float)targetAngle) / ((float)360) * ((float)angleFor360TurnLeftRotateRobot)));
        command = new Walker(pwm, -pwm); // 右に向く
        predicate = new MotorRotateAnglePredicate(angle, robotAPI->getLeftWheel());
    }
    else
    {
        angle = ((int)(((float)targetAngle) / ((float)360) * ((float)angleFor360TurnRightRotateRobot)));
        command = new Walker(-pwm, pwm); // 左に向く
        predicate = new MotorRotateAnglePredicate(-angle, robotAPI->getRightWheel());
    }

    return new CommandAndPredicate(command, predicate);
}