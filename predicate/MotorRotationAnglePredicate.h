#ifndef MotorRotationAnglePredicate_H
#define MotorRotationAnglePredicate_H

#include "Motor.h"
#include "Predicate.h"
#include "Preparizable.h"

using namespace ev3api;

// MotorRotationAnglePredicate
// モータ回転角を判定条件とするPredicate
//
// 実方
class MotorRotationAnglePredicate : public Predicate, public Preparizable
{
private:
    int angle;
    int targetAngle;
    Motor *motor;

public:
    MotorRotationAnglePredicate(int angle, Motor *motor);
    bool test() override;
    void preparation() override;
};

#endif