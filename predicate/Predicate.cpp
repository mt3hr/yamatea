#include "Predicate.h"
#include "RobotAPI.h"

Predicate::Predicate()
{
};

Predicate::~Predicate()
{
};

bool Predicate::test(RobotAPI *robotAPI)
{
    return false;
}

void Predicate::preparation(RobotAPI *robotAPI)
{
    return;
}

Predicate *Predicate::generateReversePredicate()
{
    return new Predicate();
}