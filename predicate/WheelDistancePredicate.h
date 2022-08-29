#ifndef DistancePredicate_H
#define DistancePredicate_H

#include "Predicate.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

// 距離からモータ回転角を算出する関数。単位はセンチメートル
float distanceToMotorRotateAngle(float distanceCm);

// WheelDistancePredicate
// 左車輪が指定距離（cm）分だけ回転したらtrueを返すPredicate。
// generateReversePredicate()メソッドから右車輪を対象としたDistancePredicateを生成できる。
// 主に直進するときに使える。
//
// 実方
class WheelDistancePredicate : public Predicate
{
private:
    float targetDistanceCm;
    float targetAngle = FLOAT32_MAX;
    Motor *wheel;
    bool hasLeftWheel;
    RobotAPI *robotAPI;

public:
    WheelDistancePredicate(float targetDistanceCm, RobotAPI *robotAPI);
    virtual ~WheelDistancePredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual WheelDistancePredicate *generateReversePredicate() override;
};

#endif