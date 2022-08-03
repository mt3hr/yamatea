#ifndef ExecuteOnecePredicate_H
#define ExecuteOnecePredicate_H

#include "Predicate.h"

// ExecuteOnecePredicate
// 一度だけfalseを返し、その後trueを返し続けるPredicate
//
// 実方
class ExecuteNumberOfTimesPredicate : public Predicate
{
private:
    int count;
    int currentCount = 0;

public:
    ExecuteNumberOfTimesPredicate(int count);
    bool test() override;
};

#endif