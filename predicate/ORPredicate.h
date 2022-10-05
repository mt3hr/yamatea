#ifndef ORPredicate_H
#define ORPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

class ORPredicate : public Predicate
{
private:
    Predicate *predicate1;
    Predicate *predicate2;

public:
    ORPredicate(Predicate *pradicate1, Predicate *predicate2);
    virtual ~ORPredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual ORPredicate *generateReversePredicate();
};

#endif