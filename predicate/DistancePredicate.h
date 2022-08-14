#ifndef DistancePredicate_H
#define DistancePredicate_H

#include "Predicate.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

class DistancePredicate : public Predicate
{
private:
    float targetDistanceCm;
    float targetAngle = FLOAT32_MAX;
    Motor *wheel;

public:
    DistancePredicate(float targetDistanceCm, Motor *leftWheelOrRightWheel);
    bool test(RobotAPI *robotAPI) override;
    void preparation(RobotAPI *robotAPI) override;
};

#endif