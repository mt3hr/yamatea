#include "NumberOfTimesPredicate.h"
#include "RobotAPI.h"

NumberOfTimesPredicate::NumberOfTimesPredicate(int tc)
{
    targetCount = tc;
};

NumberOfTimesPredicate::~NumberOfTimesPredicate(){};

bool NumberOfTimesPredicate::test(RobotAPI *robotAPI)
{
    return currentCount++ >= targetCount;
}

void NumberOfTimesPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

NumberOfTimesPredicate *NumberOfTimesPredicate::generateReversePredicate()
{
    return new NumberOfTimesPredicate(targetCount);
}