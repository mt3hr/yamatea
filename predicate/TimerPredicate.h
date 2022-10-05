#ifndef TimerPredicate_H
#define TimerPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"
#include "Clock.h"

using namespace ev3api;

// TimerPredicate
// 指定マイクロ秒経過したらtrueを返すPredicate。
//
// 実方
class TimerPredicate : public Predicate
{
private:
    uint64_t durationUsec = 0;
    uint64_t targetTime = 0;

public:
    TimerPredicate(uint64_t durationUsec);
    virtual ~TimerPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual TimerPredicate *generateReversePredicate() override;
};

#endif