#include "FacingAngle.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"
#include "DebugUtil.h"

FacingAngle::FacingAngle(int pwm, int targetAngle, bool useGyro)
{
    this->pwm = pwm;
    this->targetAngle = targetAngle;
    this->useGyro = useGyro;

    turnLeft = new Walker(-pwm, pwm);
    turnRight = new Walker(pwm, -pwm);
};

FacingAngle::~FacingAngle(){};

void FacingAngle::run(RobotAPI *robotAPI)
{
    int angle;
    if (useGyro)
    {
#ifndef SimulatorMode
        angle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
        angle = robotAPI->getGyroSensor()->getAngle();
#endif
    }
    else
    {
        angle = robotAPI->getMeasAngle()->getAngle();
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
    return new FacingAngle(pwm, -targetAngle, useGyro);
}

bool FacingAngle::isFinished()
{
    return finish;
}
