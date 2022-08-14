#ifndef MotorCountPredicate_H
#define MotorCountPredicate_H

#include "Predicate.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

// MotorCountPredicate
// モータ回転数がある値を超えたらtrueを返すPredicate
//
// 実方
class MotorCountPredicate : public Predicate
{
private:
    Motor *motor;
    int count;
    bool decrease;

public:
    MotorCountPredicate(Motor *motor, int count, bool decrease);
    bool test(RobotAPI *robotAPI) override;
    void preparation(RobotAPI *robotAPI) override;
};
#endif