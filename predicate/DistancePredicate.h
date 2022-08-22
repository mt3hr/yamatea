#ifndef DistancePredicate_H
#define DistancePredicate_H

#include "Predicate.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

// DistancePredicate 
// 左車輪が指定距離（cm）分だけ回転したらtrueを返すPredicate。
// generateReversePredicate()メソッドから右車輪を対象としたDistancePredicateを生成できる。
// 主に直進するときに使える。
// 
// 実方
class DistancePredicate : public Predicate
{
private:
    float targetDistanceCm;
    float targetAngle = FLOAT32_MAX;
    Motor *wheel;
    bool hasLeftWheel;
    RobotAPI *robotAPI;

public:
    DistancePredicate(float targetDistanceCm, RobotAPI *robotAPI);
    virtual ~DistancePredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual DistancePredicate *generateReversePredicate() override;
};

#endif