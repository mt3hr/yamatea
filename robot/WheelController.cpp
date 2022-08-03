#include "WheelController.h"
#include "Motor.h"

using namespace ev3api;

WheelController::WheelController(Motor *lw, Motor *rw)
{
    leftWheel = lw;
    rightWheel = rw;
}

Motor *WheelController::getLeftWheel()
{
    return leftWheel;
}

Motor *WheelController::getRightWheel()
{
    return rightWheel;
}

void WheelController::setLeftWheelPWM(int pwm)
{
    leftWheel->setPWM(pwm);
}

void WheelController::setRightWheelPWM(int pwm)
{
    rightWheel->setPWM(pwm);
}