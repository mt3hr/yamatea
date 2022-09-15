#include "Hedgehog.h"
#include "Stopper.h"
#include "RobotAPI.h"
#include "FinishConfirmable.h"

Hedgehog::Hedgehog(int targetDistance, int pwm)
{
    this->targetDistance = targetDistance;
    this->pwm = pwm;
    stopper = new Stopper();
};

Hedgehog::~Hedgehog()
{
    delete stopper;
};

void Hedgehog::run(RobotAPI *robotAPI)
{
    distance = robotAPI->getSonarSensor()->getDistance();
    if (distance == targetDistance)
    {
        stopper->run(robotAPI);
    }
    else if (distance > targetDistance)
    {
        robotAPI->getLeftWheel()->setPWM(pwm);
        robotAPI->getRightWheel()->setPWM(pwm);
    }
    else if (distance < targetDistance)
    {
        robotAPI->getLeftWheel()->setPWM(-pwm);
        robotAPI->getRightWheel()->setPWM(-pwm);
    }
}

void Hedgehog::preparation(RobotAPI *robotAPI)
{
    distance = robotAPI->getSonarSensor()->getDistance();
}

Hedgehog *Hedgehog::generateReverseCommand()
{
    return new Hedgehog(targetDistance, pwm);
}

bool Hedgehog::isFinished()
{
    return distance == targetDistance;
}