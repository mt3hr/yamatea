#ifndef GyroRotateAnglePredicate_H
#define GyroRotateAnglePredicate_H

#include "Predicate.h"

class GyroRotateAnglePredicate : public Predicate
{
private:
    int angle;
    int targetAngle;
    bool clockwise;

public:
    GyroRotateAnglePredicate(int angle);
    virtual ~GyroRotateAnglePredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual GyroRotateAnglePredicate *generateReversePredicate() override;
};

#endif