#ifndef GyroRotateAnglePredicate_H
#define GyroRotateAnglePredicate_H

#include "Predicate.h"

class GyroRotateAnglePredicate : public Predicate
{
private:
    int angle;
    int targetAngle;
    bool decrease;

public:
    GyroRotateAnglePredicate(int angle, bool decrease);
    bool test(RobotAPI *robotAPI) override;
    void preparation(RobotAPI *robotAPI) override;
};

#endif