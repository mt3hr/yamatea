#ifndef LeftWheelCountPredicate_H
#define LeftWheelCountPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

class LeftWheelCountPredicate : public Predicate
{
private:
    int count;
    bool decrease;

public:
    LeftWheelCountPredicate(int count);
    bool test(RobotAPI *robotAPI) override;
    void preparation(RobotAPI *robotAPI) override;
    Predicate *generateReversePredicate() override;
};

#endif