#ifndef FacingRobotUseWheelPredicate_H
#define FacingRobotUseWheelPredicate_H

#include "Predicate.h"

// FafingRobotUseWheelPredicate
// ロボットが指定された角度を向いていればtrueを返すPredicate。
// 角度の算出には車輪回転数を用いる。
// ジャイロ信者はGyroRotateAnglePredicateを使って。
// （ジャイロは2度ぐらい誤差が生じるよ。しかも走行していると積もっていって全く当てにならなくなる）
// （ただ、こちらのクラスは、車輪がスリップしたときに当てにならなくなるという問題がある。）
//
// 実方
class FacingRobotUseWheelPredicate : public Predicate
{
private:
    int angle;
    int targetAngle;
    bool clockwise = false;

public:
    FacingRobotUseWheelPredicate(int angle);
    virtual ~FacingRobotUseWheelPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual FacingRobotUseWheelPredicate *generateReversePredicate() override;
};

#endif