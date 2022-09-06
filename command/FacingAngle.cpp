#include "FacingAngle.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"

FacingAngle::FacingAngle(int pwm, int targetAngle)
{
    this->pwm = pwm;
    this->targetAngle = targetAngle;

    turnLeft = new Walker(-pwm, pwm);
    turnRight = new Walker(pwm, -pwm);
};

FacingAngle::~FacingAngle(){};

void FacingAngle::run(RobotAPI *robotAPI)
{
#ifndef SimulatorMode
    int angle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    int angle = robotAPI->getGyroSensor()->getAngle();
#endif
    if (angle == targetAngle)
    {
        finish = true;
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
    }
    else if (angle > targetAngle)
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
    return new FacingAngle(pwm, -targetAngle);
}

bool FacingAngle::isFinished()
{
    return finish;
}
