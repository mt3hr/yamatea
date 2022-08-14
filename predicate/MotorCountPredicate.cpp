#include "MotorCountPredicate.h"
#include "RobotAPI.h"

MotorCountPredicate::MotorCountPredicate(Motor *m, int c, bool decrease)
{
    motor = m;
    count = c;
    this->decrease = decrease;
}

bool MotorCountPredicate::test(RobotAPI *robotAPI)
{
    if (decrease)
    {
        return motor->getCount() <= count;
    }
    else
    {
        return motor->getCount() >= count;
    }
}

void MotorCountPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}