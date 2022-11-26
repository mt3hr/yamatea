#include "MotorCountPredicate.h"
#include "RobotAPI.h"

MotorCountPredicate::MotorCountPredicate(Motor *m, int c)
{
    motor = m;
    count = c;
};

MotorCountPredicate::~MotorCountPredicate()
{
};

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
    int currentCount = motor->getCount();
    decrease = count < currentCount;
}

MotorCountPredicate *MotorCountPredicate::generateReversePredicate()
{
    return new MotorCountPredicate(motor, count);
}