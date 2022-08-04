#include "NumberOfTimesPredicate.h"

NumberOfTimesPredicate::NumberOfTimesPredicate(int tc)
{
    targetCount = c;
}

bool NumberOfTimesPredicate::test()
{
    return currentCount++ == targetCount;
}