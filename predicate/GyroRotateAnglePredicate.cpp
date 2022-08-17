#include "GyroRotateAnglePredicate.h"
#include "RobotAPI.h"
#include "DebugUtil.h"

GyroRotateAnglePredicate::GyroRotateAnglePredicate(int angle, bool decrease)
{
    this->angle = angle;
    this->decrease = decrease;
}

bool GyroRotateAnglePredicate::test(RobotAPI *robotAPI)
{
    int gyroAngle = robotAPI->getGyroSensor()->getAngle();
    gyroAngle *= -1; // 分度器で測りやすくするために
    writeDebug("gyroAngle: ");
    writeDebug(gyroAngle);
    flushDebug(TRACE, robotAPI);
    if (decrease)
    {
        return gyroAngle <= targetAngle;
    }
    else
    {
        return gyroAngle >= targetAngle;
    }
}

void GyroRotateAnglePredicate::preparation(RobotAPI *robotAPI)
{
    targetAngle = robotAPI->getGyroSensor()->getAngle() + angle;
}