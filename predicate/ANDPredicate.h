#ifndef ANDPredicate_H
#define ANDPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

class ANDPredicate : public Predicate
{
private:
    Predicate *predicate1;
    Predicate *predicate2;

public:
    ANDPredicate(Predicate *predicate1, Predicate *predicate2);
    virtual ~ANDPredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual ANDPredicate *generateReversePredicate();
};

#endif