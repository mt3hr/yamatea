#include "ExecuteNumberOfTimesPredicate.h"

ExecuteNumberOfTimesPredicate::ExecuteNumberOfTimesPredicate(int c)
{
    count = c;
}

bool ExecuteNumberOfTimesPredicate::test()
{
    return ++currentCount == count; // なんか前置インクリメントしないと1回多く実行されるんだけど！！なんで！
}