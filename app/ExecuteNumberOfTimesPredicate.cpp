#include "ExecuteNumberOfTimesPredicate.h"

ExecuteNumberOfTimesPredicate::ExecuteNumberOfTimesPredicate(int c)
{
    count = c;
}

bool ExecuteNumberOfTimesPredicate::test()
{
    return currentCount++ == count;
}