#include "Command.h"
#include "Predicate.h"
#include "CommandAndPredicate.h"
#include "Walker.h"
#include "WheelController.h"
#include "MotorRotationAnglePredicate.h"

CommandAndPredicate *generateRotationRobotCommand(int targetAngle, WheelController *wheelController)
{
    int angleFor360Turn = 10; // TODO 360度旋回するのに必要な左右車輪回転角度数

    int pwm = 10;
    int angle = ((int)((float)targetAngle) / ((float)angleFor360Turn) * ((float)360));
    Command *command;
    Predicate *predicate ;
    if (targetAngle > 0)
    {
        command = new Walker(pwm, -pwm, wheelController); // 右に向く
    predicate = new MotorRotationAnglePredicate(angle, wheelController->getLeftWheel());
    }
    else
    {
        command = new Walker(-pwm, pwm, wheelController); // 左に向く
    predicate = new MotorRotationAnglePredicate(angle, wheelController->getRightWheel());
    }


    

    return new CommandAndPredicate(command, predicate);
}