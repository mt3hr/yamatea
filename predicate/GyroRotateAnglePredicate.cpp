#include "GyroRotateAnglePredicate.h"
#include "RobotAPI.h"

GyroRotateAnglePredicate::GyroRotateAnglePredicate(int angle)
{
    this->angle = angle;
}

bool GyroRotateAnglePredicate::test(RobotAPI *robotAPI)
{
    return robotAPI->getGyroSensor()->getAngle() > targetAngle;
}

void GyroRotateAnglePredicate::preparation(RobotAPI *robotAPI)
{
    targetAngle = robotAPI->getGyroSensor()->getAngle() + angle;
}