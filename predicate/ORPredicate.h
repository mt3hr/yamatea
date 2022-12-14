#ifndef ORPredicate_H
#define ORPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"
// ORPredicate
// 2つのPredicateのうち1つがtrueを返したときにtrueを返すPredicate
//
// 実方
class ORPredicate : public Predicate
{
private:
    Predicate *predicate1;
    Predicate *predicate2;

public:
    ORPredicate(Predicate *pradicate1, Predicate *predicate2);
    virtual ~ORPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ORPredicate *generateReversePredicate() override;
};

#endif