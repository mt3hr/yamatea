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
    virtual ~RightWheelCountPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Predicate *generateReversePredicate() override;
};

#endif