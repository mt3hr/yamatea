#ifndef MotorRotateAnglePredicate_H
#define MotorRotateAnglePredicate_H

#include "Motor.h"
#include "Predicate.h"
#include "Preparizable.h"

using namespace ev3api;

// MotorRotateAnglePredicate
// モータ回転角を判定条件とするPredicate
//
// 実方
class MotorRotateAnglePredicate : public Predicate, public Preparizable
{
private:
    int angle;       // コンストラクタ引数から渡される角度の値
    int targetAngle; // preparation()メソッド実行時に決定される角度の値。angleにこれまでに回転したモータ角度を加算したもの。
    Motor *motor;

public:
    MotorRotateAnglePredicate(int angle, Motor *motor);
    bool test() override;
    void preparation() override;
};

#endif