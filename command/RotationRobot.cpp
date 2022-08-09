#include "Command.h"
#include "Predicate.h"
#include "CommandAndPredicate.h"
#include "Walker.h"
#include "WheelController.h"
#include "MotorRotationAnglePredicate.h"
#include "ExecutePreparationWhenExitBeforeCommandHandler.h"
#include "Setting.h"

CommandAndPredicate *generateRotationRobotCommand(int targetAngle, int pwm, WheelController *wheelController)
{
    int angle;
    Command *command;
    MotorRotationAnglePredicate *predicate;
    Handler *preHandler;
    if (targetAngle > 0)
    {
        angle = ((int)(((float)targetAngle) / ((float)360) * ((float)angleFor360TurnLeftRotateRobot)));
        command = new Walker(pwm, -pwm, wheelController); // 右に向く
        predicate = new MotorRotationAnglePredicate(angle, wheelController->getLeftWheel());
        preHandler = new ExecutePreparationWhenExitBeforeCommandHandler(predicate);
    }
    else
    {
        angle = ((int)(((float)targetAngle) / ((float)360) * ((float)angleFor360TurnRightRotateRobot)));
        command = new Walker(-pwm, pwm, wheelController); // 左に向く
        predicate = new MotorRotationAnglePredicate(-angle, wheelController->getRightWheel());
        preHandler = new ExecutePreparationWhenExitBeforeCommandHandler(predicate);
    }

    return new CommandAndPredicate(command, predicate, preHandler);
}