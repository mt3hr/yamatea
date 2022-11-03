#ifndef ANDPredicate_H
#define ANDPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

// ANDPredicate
// 2つのPredicateがtrueを返したときにtrueを返すPredicate
//
// 実方
class ANDPredicate : public Predicate
{
private:
    Predicate *predicate1;
    Predicate *predicate2;

public:
    ANDPredicate(Predicate *predicate1, Predicate *predicate2);
    virtual ~ANDPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ANDPredicate *generateReversePredicate() override;
};

#endif