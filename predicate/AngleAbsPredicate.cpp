#include "AngleAbsPredicate.h"
#include "RobotAPI.h"

AngleAbsPredicate::AngleAbsPredicate(AngleAbsPredicateMode mode, int angle, bool up)
{
    this->mode = mode;
    this->angle = angle;
    this->up = up;
};

AngleAbsPredicate::~AngleAbsPredicate(){};

bool AngleAbsPredicate::test(RobotAPI *robotAPI)
{
    switch (mode)
    {
    case AAPM_Gyro:
    {
        if (up)
        {
            return angle >= robotAPI->getGyroSensor()->getAngle();
        }
        else
        {
            return angle <= robotAPI->getGyroSensor()->getAngle();
        }
    }
    case AAPM_WheelCount:
    {
        if (up)
        {
            return angle >= robotAPI->getMeasAngle()->getAngle();
        }
        else
        {
            return angle <= robotAPI->getMeasAngle()->getAngle();
        }
    }
    default:
        return false;
        break;
    }
}

void AngleAbsPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

AngleAbsPredicate *AngleAbsPredicate::generateReversePredicate()
{
    return new AngleAbsPredicate(mode, -angle, up);
}