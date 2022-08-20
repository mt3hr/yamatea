#ifndef DistancePredicate_H
#define DistancePredicate_H

#include "Predicate.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

// 直進するときに使える。
class DistancePredicate : public Predicate
{
private:
    float targetDistanceCm;
    float targetAngle = FLOAT32_MAX;
    Motor *wheel;
    bool hasLeftMotor;
    RobotAPI *robotAPI;

public:
    DistancePredicate(float targetDistanceCm, RobotAPI *robotAPI);
    virtual ~DistancePredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual DistancePredicate *generateReversePredicate() override;
};

#endif