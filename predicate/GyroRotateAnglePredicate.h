#ifndef GyroRotateAnglePredicate_H
#define GyroRotateAnglePredicate_H

#include "Predicate.h"

class GyroRotateAnglePredicate : public Predicate
{
private:
    int angle;
    int targetAngle;

public:
    GyroRotateAnglePredicate(int angle);
    bool test(RobotAPI *robotAPI) override;
    void preparation(RobotAPI *robotAPI) override;
};

#endif