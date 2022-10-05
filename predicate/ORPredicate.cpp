#include "ORPredicate.h"
#include "Predicate.h"
#include "RobotAPI.h"

ORPredicate::ORPredicate(Predicate *pradicate1, Predicate *predicate2)
{
    this->predicate1 = predicate1;
    this->predicate2 = predicate2;
}

ORPredicate::~ORPredicate()
{
}

bool ORPredicate::test(RobotAPI *robotAPI)
{
    return predicate1->test(robotAPI) || predicate2->test(robotAPI);
}

void ORPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

ORPredicate *ORPredicate::generateReversePredicate()
{
    return new ORPredicate(predicate1, predicate2);
}