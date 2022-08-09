#include "Motor.h"
#include "MotorRotateAnglePredicate.h"

using namespace ev3api;

MotorRotateAnglePredicate::MotorRotateAnglePredicate(int a, Motor *m)
{
    angle = a;
    motor = m;
}

bool MotorRotateAnglePredicate::test()
{
    return motor->getCount() > targetAngle;
}

void MotorRotateAnglePredicate::preparation()
{
    targetAngle = motor->getCount() + angle;
}