#include "GyroRotateAnglePredicate.h"
#include "RobotAPI.h"
#include "DebugUtil.h"

// +で時計回り、-で反時計回り
GyroRotateAnglePredicate::GyroRotateAnglePredicate(int angle)
{
    this->angle = angle;
    clockwise = angle > 0;
}

bool GyroRotateAnglePredicate::test(RobotAPI *robotAPI)
{
    int gyroAngle = robotAPI->getGyroSensor()->getAngle();

    writeDebug("gyroAngle: ");
    writeDebug(gyroAngle);
    flushDebug(TRACE, robotAPI);

    if (clockwise)
    {
        return gyroAngle >= targetAngle;
    }
    else
    {
        return gyroAngle <= targetAngle;
    }
}

void GyroRotateAnglePredicate::preparation(RobotAPI *robotAPI)
{
    targetAngle = (robotAPI->getGyroSensor()->getAngle() + angle);
    writeDebug("GyroRotateAnglePredicate.preparation().targetAngle: ");
    writeDebug(targetAngle);
    flushDebug(DEBUG, robotAPI);
}