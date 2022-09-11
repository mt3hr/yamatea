#include "FacingRobotUseWheelPredicate.h"

FacingRobotUseWheelPredicate::FacingRobotUseWheelPredicate(int angle)
{
    this->angle = angle;
    this->clockwise = angle > 0;
};

FacingRobotUseWheelPredicate::~FacingRobotUseWheelPredicate(){};

bool FacingRobotUseWheelPredicate::test(RobotAPI *robotAPI)
{
    int currentAngle = robotAPI->getMeasAngle()->getAngle();
    if (clockwise)
    {
        return currentAngle >= targetAngle;
    }
    else
    {
        return currentAngle <= targetAngle;
    }
}

void FacingRobotUseWheelPredicate::preparation(RobotAPI *robotAPI)
{
    this->targetAngle = robotAPI->getMeasAngle()->getAngle() + angle;
}

FacingRobotUseWheelPredicate *FacingRobotUseWheelPredicate::generateReversePredicate()
{
    return new FacingRobotUseWheelPredicate(-angle);
}