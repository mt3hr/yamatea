#include "Motor.h"
#include "MotorRotateAnglePredicate.h"
#include "RobotAPI.h"

using namespace ev3api;

MotorRotateAnglePredicate::MotorRotateAnglePredicate(int a, Motor *m)
{
    angle = a;
    motor = m;
}

bool MotorRotateAnglePredicate::test(RobotAPI *robotAPI)
{
    return motor->getCount() > targetAngle;
}

void MotorRotateAnglePredicate::preparation()
{
    targetAngle = motor->getCount() + angle;
}