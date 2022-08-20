#include "RightWheelCountPredicate.h"
#include "LeftWheelCountPredicate.h"
#include "RobotAPI.h"

RightWheelCountPredicate::RightWheelCountPredicate(int count)
{
    this->count = count;
}

RightWheelCountPredicate::~RightWheelCountPredicate()
{
}

bool RightWheelCountPredicate::test(RobotAPI *robotAPI)
{
    if (!decrease)
    {
        return count <= robotAPI->getLeftWheel()->getCount();
    }
    else
    {
        return count >= robotAPI->getLeftWheel()->getCount();
    }
}

void RightWheelCountPredicate::preparation(RobotAPI *robotAPI)
{
    int currentCount = robotAPI->getRightWheel()->getCount();
    decrease = count < currentCount;
}

Predicate *RightWheelCountPredicate::generateReversePredicate()
{
    return new LeftWheelCountPredicate(count);
}