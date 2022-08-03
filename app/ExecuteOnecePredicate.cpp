#include "ExecuteOnecePredicate.h"

ExecuteOnecePredicate::ExecuteOnecePredicate()
{
}

bool ExecuteOnecePredicate::test()
{
    if (!executed)
    {
        executed = true;
        return false;
    }
    return true;
}