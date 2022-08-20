#include "LeftWheelCountPredicate.h"
#include "RightWheelCountPredicate.h"
#include "RobotAPI.h"

LeftWheelCountPredicate::LeftWheelCountPredicate(int count)
{
    this->count = count;
}

bool LeftWheelCountPredicate::test(RobotAPI *robotAPI)
{
    if (!decrease)
    {
        return count <= robotAPI->getRightWheel()->getCount();
    }
    else
    {
        return count >= robotAPI->getRightWheel()->getCount();
    }
}

void LeftWheelCountPredicate::preparation(RobotAPI *robotAPI)
{
    int currentCount = robotAPI->getRightWheel()->getCount();
    decrease = count < currentCount;
}

Predicate *LeftWheelCountPredicate::generateReversePredicate()
{
    return new RightWheelCountPredicate(count);
}