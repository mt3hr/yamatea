#include "ANDPredicate.h"
#include "Predicate.h"
#include "RobotAPI.h"

ANDPredicate::ANDPredicate(Predicate *predicate1, Predicate *predicate2)
{
    this->predicate1 = predicate1;
    this->predicate2 = predicate2;
}

ANDPredicate::~ANDPredicate()
{
}

bool ANDPredicate::test(RobotAPI *robotAPI)
{
    return predicate1->test(robotAPI) && predicate2->test(robotAPI);
}

void ANDPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

ANDPredicate *ANDPredicate::generateReversePredicate()
{
    return new ANDPredicate(predicate1, predicate2);
}

