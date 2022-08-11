#include "MotorCountPredicate.h"

MotorCountPredicate::MotorCountPredicate(Motor *m, int c, bool decrease)
{
    motor = m;
    count = c;
    this->decrease = decrease;
}

bool MotorCountPredicate::test()
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