#ifndef TimerPredicate_H
#define TimerPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"
#include "Clock.h"

using namespace ev3api;

class TimerPredicate : public Predicate
{
private:
    uint64_t durationUsec;
    uint64_t targetTime;

public:
    TimerPredicate(uint64_t durationUsec);
    virtual ~TimerPredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual TimerPredicate *generateReversePredicate();
};

#endif