#include "HedgehogUsePID.h"
#include "Stopper.h"
#include "RobotAPI.h"
#include "FinishConfirmable.h"

HedgehogUsePID::HedgehogUsePID(int targetDistance, float pwm, float kp, float ki, float kd, float dt)
{
    this->targetDistance = targetDistance;
    this->pwm = pwm;
    stopper = new Stopper();

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;

    walker = new PIDStraightWalker(pwm, kp, ki, kd, dt);
    backer = new PIDStraightWalker(-pwm, kp, ki, kd, dt);
    state = HHUPS_INIT;
};

HedgehogUsePID::~HedgehogUsePID()
{
    delete stopper;
};

void HedgehogUsePID::run(RobotAPI *robotAPI)
{
    distance = robotAPI->getSonarSensor()->getDistance();
    if (distance == targetDistance)
    {
        stopper->run(robotAPI);
    }
    else if (distance > targetDistance)
    {
       walker->run(robotAPI);
    }
    else if (distance < targetDistance)
    {
       backer->run(robotAPI);
    }
}

void HedgehogUsePID::preparation(RobotAPI *robotAPI)
{
    distance = robotAPI->getSonarSensor()->getDistance();
    walker->setTargetDifferenceWheelCount(robotAPI->getLeftWheel()->getCount() - robotAPI->getRightWheel()->getCount());
    backer->setTargetDifferenceWheelCount(robotAPI->getLeftWheel()->getCount() - robotAPI->getRightWheel()->getCount());
}

HedgehogUsePID *HedgehogUsePID::generateReverseCommand()
{
    return new HedgehogUsePID(targetDistance, pwm, kp, ki, kd, dt);
}

bool HedgehogUsePID::isFinished()
{
    return distance == targetDistance;
}