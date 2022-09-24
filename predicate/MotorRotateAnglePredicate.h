#ifndef MotorRotateAnglePredicate_H
#define MotorRotateAnglePredicate_H

#include "CorrectedMotor.h"
#include "Predicate.h"
#include "RobotAPI.h"

using namespace ev3api;

// MotorRotateAnglePredicate
// モータ回転角を判定条件とするPredicate
// MotorRotateAnglePredicate: 指定された角度分車輪を回転されたかどうかで判定。
// MotorCountPredicate: モータの累積回転数で判定
//
// 実方
class MotorRotateAnglePredicate : public Predicate
{
private:
    int angle;       // コンストラクタ引数から渡される角度の値
    int targetAngle; // preparation()メソッド実行時に決定される角度の値。angleにこれまでに回転したモータ角度を加算したもの。
    CorrectedMotor *motor;
    bool up = false;

public:
    MotorRotateAnglePredicate(int angle, CorrectedMotor *motor);
    virtual ~MotorRotateAnglePredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual MotorRotateAnglePredicate *generateReversePredicate() override;
};

#endif