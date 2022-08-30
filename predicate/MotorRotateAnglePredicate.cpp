#include "Motor.h"
#include "MotorRotateAnglePredicate.h"
#include "RobotAPI.h"

using namespace ev3api;

MotorRotateAnglePredicate::MotorRotateAnglePredicate(int a, Motor *m)
{
    angle = a;
    motor = m;
    up = a > 0;
}

MotorRotateAnglePredicate::~MotorRotateAnglePredicate()
{
}

bool MotorRotateAnglePredicate::test(RobotAPI *robotAPI)
{
    if (up)
    {
        return motor->getCount() > targetAngle;
    }
    else
    {
        return motor->getCount() < targetAngle;
    }
}

void MotorRotateAnglePredicate::preparation(RobotAPI *robotAPI)
{
    targetAngle = motor->getCount() + angle;
}

MotorRotateAnglePredicate *MotorRotateAnglePredicate::generateReversePredicate()
{
    return new MotorRotateAnglePredicate(angle, motor);
}