#ifndef DistancePredicate_H
#define DistancePredicate_H

#include "Predicate.h"
#include "CorrectedMotor.h"
#include "RobotAPI.h"

using namespace ev3api;

// 距離からモータ回転角を算出する関数。単位はセンチメートル
float distanceToMotorRotateAngle(float distanceCm);

// WheelDistancePredicate
// 左車輪が指定距離（cm）分だけ回転したらtrueを返すPredicate。
// generateReversePredicate()メソッドから右車輪を対象としたDistancePredicateを生成できる。
// 主に直進するとき、カーブ走行をするときに使える。
// カーブ走行はcommand_and_predicate/CurvatureWalkerCommandAndPredicateを使えば実現できるけど
//
// 実方
class WheelDistancePredicate : public Predicate
{
private:
    float targetDistanceCm;
    float targetAngle = FLOAT32_MAX;
    CorrectedMotor *wheel;
    bool hasLeftWheel = true;
    RobotAPI *robotAPI;
    bool up = false;

public:
    WheelDistancePredicate(float targetDistanceCm);
    virtual ~WheelDistancePredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual WheelDistancePredicate *generateReversePredicate() override;
};

#endif