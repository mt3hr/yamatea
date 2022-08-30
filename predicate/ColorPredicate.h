#ifndef ColorPredicate_H
#define ColorPredicate_H

#include "Predicate.h"
#include "Sensor.h"
#include "RobotAPI.h"

class ColorPredicate : public Predicate
{
private:
    colorid_t colorID;

public:
    ColorPredicate(colorid_t colorID);
    virtual ~ColorPredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual Predicate *generateReversePredicate();
};
#endif