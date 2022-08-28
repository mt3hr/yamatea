#include "DistancePredicate.h"
#include "Setting.h"
#include "math.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

float distanceToMotorRotateAngle(float distanceCm)
{
    float circumference = wheelDiameter * M_PI; // 円周
    float distanceOf360 = circumference;        // 360度回転したときに進む距離
    float cm1Angle = 360 / distanceOf360;       // 1cm進むときのモータ回転角
    return cm1Angle * distanceCm;
};

DistancePredicate::DistancePredicate(float tdc, RobotAPI *robotAPI)
{
    targetDistanceCm = tdc;
    wheel = robotAPI->getLeftWheel();
    hasLeftWheel = true;
    this->robotAPI = robotAPI;
};

DistancePredicate::~DistancePredicate(){};

bool DistancePredicate::test(RobotAPI *robotAPI)
{
    return wheel->getCount() > targetAngle;
}

void DistancePredicate::preparation(RobotAPI *robotAPI)
{
    targetAngle = distanceToMotorRotateAngle(targetDistanceCm) + float(wheel->getCount());
}

DistancePredicate *DistancePredicate::generateReversePredicate()
{
    DistancePredicate *reversed = new DistancePredicate(targetDistanceCm, robotAPI);
    if (reversed->hasLeftWheel)
    {
        reversed->wheel = robotAPI->getRightWheel();
        reversed->hasLeftWheel = false;
    }
    else
    {
        reversed->wheel = robotAPI->getLeftWheel();
        reversed->hasLeftWheel = true;
    }
    return reversed;
}
