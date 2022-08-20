#include "GyroRotateAnglePredicate.h"
#include "RobotAPI.h"
#include "DebugUtil.h"
#include "Setting.h"

// +で時計回り、-で反時計回り
GyroRotateAnglePredicate::GyroRotateAnglePredicate(int angle)
{
    this->angle = angle;
    clockwise = angle > 0;
};

GyroRotateAnglePredicate::~GyroRotateAnglePredicate()
{
};

bool GyroRotateAnglePredicate::test(RobotAPI *robotAPI)
{
    int gyroAngle = robotAPI->getGyroSensor()->getAngle();

#ifndef SimulatorMode
    gyroAngle *= -1;
#endif

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
    float currentAngle = robotAPI->getGyroSensor()->getAngle();

#ifndef SimulatorMode
    currentAngle *= -1;
#endif

    targetAngle = (currentAngle + angle);
    writeDebug("GyroRotateAnglePredicate.preparation().angle: ");
    writeDebug(angle);
    writeEndLineDebug();
    writeDebug("GyroRotateAnglePredicate.preparation().targetAngle: ");
    writeDebug(targetAngle);
    flushDebug(DEBUG, robotAPI);
}

GyroRotateAnglePredicate *GyroRotateAnglePredicate::generateReversePredicate()
{
    return new GyroRotateAnglePredicate(-angle);
}