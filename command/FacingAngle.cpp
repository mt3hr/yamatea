#include "FacingAngle.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"
#include "DebugUtil.h"
#include "FinishConfirmable.h"

FacingAngle::FacingAngle(FacingAngleMode mode, int pwm, int angle)
{
    this->mode = mode;
    this->pwm = pwm;
    this->angle = angle;

    turnLeft = new Walker(-pwm, pwm);
    turnRight = new Walker(pwm, -pwm);
};

FacingAngle::~FacingAngle(){};

void FacingAngle::run(RobotAPI *robotAPI)
{
    int currentAngle = 0;
    switch (mode)
    {
    case FA_Gyro:
    {
#ifndef SimulatorMode
        currentAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
        currentAngle = robotAPI->getGyroSensor()->getAngle();
#endif
        break;
    }
    case FA_WheelCount:
    {
        currentAngle = robotAPI->getMeasAngle()->getAngle();
        break;
    }
    default:
        break;
    }

    writeDebug("FacingAngle");
    writeEndLineDebug();
    writeDebug("angle: ");
    writeDebug(currentAngle);
    flushDebug(TRACE, robotAPI);

    if (currentAngle == targetAngle)
    {
        finish = true;
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
    }
    else if (currentAngle < targetAngle)
    {
        turnRight->run(robotAPI);
    }
    else
    {
        turnLeft->run(robotAPI);
    }
}

void FacingAngle::preparation(RobotAPI *robotAPI)
{
    int currentAngle = 0;
    switch (mode)
    {
    case FA_Gyro:
    {
#ifndef SimulatorMode
        currentAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
        currentAngle = robotAPI->getGyroSensor()->getAngle();
#endif
        break;
    }
    case FA_WheelCount:
    {
        currentAngle = robotAPI->getMeasAngle()->getAngle();
        break;
    }
    default:
        break;
    }
    targetAngle = currentAngle + angle;
    return;
}

FacingAngle *FacingAngle::generateReverseCommand()
{
    return new FacingAngle(mode, pwm, -angle);
}

bool FacingAngle::isFinished()
{
    return finish;
}
