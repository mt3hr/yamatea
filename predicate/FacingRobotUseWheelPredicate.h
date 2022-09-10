#ifndef FacingRobotUseWheelPredicate_H
#define FacingRobotUseWheelPredicate_H

#include "Predicate.h"

class FacingRobotUseWheelPredicate : public Predicate
{
private:
    int angle;
    int targetAngle;
    bool clockwise = false;

public:
    FacingRobotUseWheelPredicate(int angle);
    virtual ~FacingRobotUseWheelPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual FacingRobotUseWheelPredicate *generateReversePredicate() override;
};


#endif