#include "NumberOfTimesPredicate.h"
#include "RobotAPI.h"

NumberOfTimesPredicate::NumberOfTimesPredicate(int tc)
{
    targetCount = tc;
}

bool NumberOfTimesPredicate::test(RobotAPI *robotAPI)
{
    return currentCount++ == targetCount;
}

void NumberOfTimesPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}