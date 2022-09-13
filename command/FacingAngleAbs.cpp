#include "FacingAngleAbs.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"
#include "DebugUtil.h"

FacingAngleAbs::FacingAngleAbs(FacingAngleMode mode, int pwm, int targetAngle)
{
    this->mode = mode;
    this->pwm = pwm;
    this->targetAngle = targetAngle;

    turnLeft = new Walker(-pwm, pwm);
    turnRight = new Walker(pwm, -pwm);
};

FacingAngleAbs::~FacingAngleAbs(){};

void FacingAngleAbs::run(RobotAPI *robotAPI)
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

void FacingAngleAbs::preparation(RobotAPI *robotAPI)
{
    return;
}

FacingAngleAbs *FacingAngleAbs::generateReverseCommand()
{
    return new FacingAngleAbs(mode, pwm, -targetAngle);
}

bool FacingAngleAbs::isFinished()
{
    return finish;
}
