#include "DistancePredicate.h"
#include "Setting.h"
#include "math.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

DistancePredicate::DistancePredicate(float tdc, RobotAPI *robotAPI)
{
    targetDistanceCm = tdc;
    wheel = robotAPI->getLeftWheel();
    hasLeftMotor = true;
    this->robotAPI = robotAPI;
}

DistancePredicate::~DistancePredicate()
{
}

bool DistancePredicate::test(RobotAPI *robotAPI)
{
    return wheel->getCount() > targetAngle;
}

void DistancePredicate::preparation(RobotAPI *robotAPI)
{
    float circumference = wheelDiameter * M_PI; // 円周
    float distanceOf360 = circumference;        // 360度回転したときに進む距離
    float cm1Angle = 360 / distanceOf360;       // 1cm進むときのモータ回転角
    targetAngle = (cm1Angle * targetDistanceCm) + float(wheel->getCount());
}

DistancePredicate *DistancePredicate::generateReversePredicate()
{
    DistancePredicate *reversed = new DistancePredicate(targetDistanceCm, robotAPI);
    if (reversed->hasLeftMotor)
    {
        reversed->wheel = robotAPI->getRightWheel();
    }
    else
    {
        reversed->wheel = robotAPI->getLeftWheel();
    }
    return reversed;
}