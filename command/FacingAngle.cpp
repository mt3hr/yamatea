#include "FacingAngle.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"
#include "DebugUtil.h"

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
#ifndef Gyrooooooo                                    // TODO 消して
    int angle = robotAPI->getMeasAngle()->getAngle(); // TODO メモリ管理ガバガバ
#else                                                 // TODO 消して
#ifndef SimulatorMode
    int angle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    int angle = robotAPI->getGyroSensor()->getAngle();
#endif
#endif // TODO 消して

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
    return new FacingAngle(pwm, -targetAngle);
}

bool FacingAngle::isFinished()
{
    return finish;
}
