#ifndef MotorCountPredicate_H
#define MotorCountPredicate_H

#include "Predicate.h"
#include "CorrectedMotor.h"
#include "RobotAPI.h"

using namespace ev3api;

// MotorCountPredicate
// モータ回転数がある値を超えたらtrueを返すPredicate
// MotorCountPredicate: モータの累積回転数で判定
// MotorRotateAnglePredicate: 指定された角度分車輪を回転されたかどうかで判定。
//
// 実方
class MotorCountPredicate : public Predicate
{
private:
    CorrectedMotor *motor;
    int count;
    bool decrease;

public:
    MotorCountPredicate(CorrectedMotor *motor, int count);
    virtual ~MotorCountPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual MotorCountPredicate *generateReversePredicate() override;
};
#endif