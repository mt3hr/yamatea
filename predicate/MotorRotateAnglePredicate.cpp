#include "Motor.h"
#include "MotorRotateAnglePredicate.h"
#include "RobotAPI.h"

using namespace ev3api;

MotorRotateAnglePredicate::MotorRotateAnglePredicate(int a, Motor *m)
{
    angle = a;
    motor = m;
}

MotorRotateAnglePredicate::~MotorRotateAnglePredicate()
{
}

bool MotorRotateAnglePredicate::test(RobotAPI *robotAPI)
{
    return motor->getCount() > targetAngle;
}

void MotorRotateAnglePredicate::preparation(RobotAPI *robotAPI)
{
    targetAngle = motor->getCount() + angle;
}

MotorRotateAnglePredicate *MotorRotateAnglePredicate::generateReversePredicate()
{
    return new MotorRotateAnglePredicate(angle, motor);
}