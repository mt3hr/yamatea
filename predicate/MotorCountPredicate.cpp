#ifndef MotorCountPredicate_H
#define MotorCountPredicate_H
#include "MotorCountPredicate.h"

MotorCountPredicate::MotorCountPredicate(Motor *m, int c)
{
    motor = m;
    count = c;
}

bool MotorCountPredicate::test()
{
    return motor->getCount() > count;
}
#endif