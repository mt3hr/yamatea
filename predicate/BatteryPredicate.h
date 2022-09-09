#ifndef BatteryPredicate_H
#define BatteryPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

class BatteryPredicate : public Predicate
{
private:
    int targetVoltage;

public:
    BatteryPredicate(int targetVoltage);
    virtual ~BatteryPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Predicate *generateReversePredicate() override;
};

#endif