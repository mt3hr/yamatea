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

GyroRotateAnglePredicate::~GyroRotateAnglePredicate(){};

bool GyroRotateAnglePredicate::test(RobotAPI *robotAPI)
{
#ifndef SimulatorMode
    int gyroAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    int gyroAngle = robotAPI->getGyroSensor()->getAngle();
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
#ifndef SimulatorMode
    float currentAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    float currentAngle = robotAPI->getGyroSensor()->getAngle();
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