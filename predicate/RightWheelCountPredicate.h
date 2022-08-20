#ifndef RightWheelCountPredicate_H
#define RightWheelCountPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"
#include "LeftWheelCountPredicate.h"

using namespace ev3api;

class RightWheelCountPredicate : public Predicate
{
private:
    int count;
    bool decrease;

public:
    RightWheelCountPredicate(int count);
    bool test(RobotAPI *robotAPI) override;
    void preparation(RobotAPI *robotAPI) override;
    Predicate *generateReversePredicate() override;
};

#endif