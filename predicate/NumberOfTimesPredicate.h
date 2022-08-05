#ifndef NumberOfTimesPredicate_H
#define NumberOfTimesPredicate_H

#include "Predicate.h"

// NumberOfTimesPredicate 
// 指定回数実行されるまでfalseを返し、その後trueを返すPredicate
//
// 実方
class NumberOfTimesPredicate : public Predicate
{
private:
    int targetCount;
    int currentCount = 0;

public:
    NumberOfTimesPredicate(int targetCount);
    bool test() override;
};

#endif