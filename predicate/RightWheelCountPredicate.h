#ifndef RightWheelCountPredicate_H
#define RightWheelCountPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"
#include "LeftWheelCountPredicate.h"

using namespace ev3api;

// RightWheelCountPredicate 
// 右車輪が指定された回数回転したらtrueを返すpredicate。
// 概念的にはMotorCountPredicateのサブクラス。
// 
// 実方
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