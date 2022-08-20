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
    virtual ~LeftWheelCountPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Predicate *generateReversePredicate() override;
};

#endif