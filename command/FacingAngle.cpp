#include "FacingAngle.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"
#include "DebugUtil.h"

FacingAngle::FacingAngle(FacingAngleMode mode, int pwm, int targetAngle)
{
    this->mode = mode;
    this->pwm = pwm;
    this->targetAngle = targetAngle;

    turnLeft = new Walker(-pwm, pwm);
    turnRight = new Walker(pwm, -pwm);
};

FacingAngle::~FacingAngle(){};

void FacingAngle::run(RobotAPI *robotAPI)
{
    int angle = 0;
    switch (mode)
    {
    case FA_Gyro:
    {
#ifndef SimulatorMode
        angle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
        angle = robotAPI->getGyroSensor()->getAngle();
#endif
        break;
    }
    case FA_WheelCount:
    {
        angle = robotAPI->getMeasAngle()->getAngle();
        break;
    }
    default:
        break;
    }

    writeDebug("FacingAngle");
    writeEndLineDebug();
    writeDebug("angle: ");
    writeDebug(angle);
    flushDebug(TRACE, robotAPI);

    if (angle == targetAngle)
    {
        finish = true;
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
    }
    else if (angle < targetAngle)
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
    return;
}

Command *FacingAngle::generateReverseCommand()
{
    return new FacingAngle(mode, pwm, -targetAngle);
}

bool FacingAngle::isFinished()
{
    return finish;
}
