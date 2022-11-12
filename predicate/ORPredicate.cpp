#include "ORPredicate.h"
#include "Predicate.h"
#include "RobotAPI.h"

ORPredicate::ORPredicate(Predicate *predicate1, Predicate *predicate2)
{
    this->predicate1 = predicate1;
    this->predicate2 = predicate2;
}

ORPredicate::~ORPredicate()
{
    return;
}

bool ORPredicate::test(RobotAPI *robotAPI)
{
    return predicate1->test(robotAPI) || predicate2->test(robotAPI);
}

void ORPredicate::preparation(RobotAPI *robotAPI)
{
    predicate1->preparation(robotAPI);
    predicate2->preparation(robotAPI);
}

ORPredicate *ORPredicate::generateReversePredicate()
{
    return new ORPredicate(predicate1->generateReversePredicate(), predicate2->generateReversePredicate());
}