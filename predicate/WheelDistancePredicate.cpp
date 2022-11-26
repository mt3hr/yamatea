#include "WheelDistancePredicate.h"
#include "Setting.h"
#include "math.h"
#include "CorrectedMotor.h"
#include "RobotAPI.h"

using namespace ev3api;

float distanceToMotorRotateAngle(float distanceCm)
{
    float circumference = wheelDiameter * M_PI; // 円周
    float distanceOf360 = circumference;        // 360度回転したときに進む距離
    float cm1Angle = 360 / distanceOf360;       // 1cm進むときのモータ回転角
    return cm1Angle * distanceCm;
};

WheelDistancePredicate::WheelDistancePredicate(float tdc, RobotAPI *robotAPI)
{
    targetDistanceCm = tdc;
    wheel = robotAPI->getLeftWheel();
    hasLeftWheel = true;
    this->robotAPI = robotAPI;
    up = tdc > 0;
};

WheelDistancePredicate::~WheelDistancePredicate(){};

bool WheelDistancePredicate::test(RobotAPI *robotAPI)
{
    if (up)
    {
        return wheel->getCount() >= targetAngle;
    }
    else
    {
        return wheel->getCount() <= targetAngle;
    }
}

void WheelDistancePredicate::preparation(RobotAPI *robotAPI)
{
    targetAngle = distanceToMotorRotateAngle(targetDistanceCm) + float(wheel->getCount());
}

WheelDistancePredicate *WheelDistancePredicate::generateReversePredicate()
{
    WheelDistancePredicate *reversed = new WheelDistancePredicate(targetDistanceCm, robotAPI);
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
