#ifndef ExecuteOnecePredicate_H
#define ExecuteOnecePredicate_H

#include "Predicate.h"

// ExecuteOnecePredicate
// 一度だけfalseを返し、その後trueを返し続けるPredicate
// 
// 実方
class ExecuteOnecePredicate : public Predicate
{
private:
    bool executed;

public:
    ExecuteOnecePredicate();
    bool test() override;
};

#endif