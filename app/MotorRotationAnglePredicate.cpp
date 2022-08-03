#include "Motor.h"
#include "MotorRotationAnglePredicate.h"

using namespace ev3api;

MotorRotationAnglePredicate::MotorRotationAnglePredicate(int a, Motor *m)
{
    angle = a;
    motor = m;
}

bool MotorRotationAnglePredicate::test()
{
    return motor->getCount() > targetAngle;
}

void MotorRotationAnglePredicate::preparation()
{
    targetAngle = motor->getCount() + angle;
}