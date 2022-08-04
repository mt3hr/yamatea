#include "NumberOfTimesPredicate.h"

NumberOfTimesPredicate::NumberOfTimesPredicate(int tc)
{
    targetCount = tc;
}

bool NumberOfTimesPredicate::test()
{
    return currentCount++ == targetCount;
}