#ifndef BatteryPredicate_H
#define BatteryPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

// BatteryPredicate
// バッテリーが指定されたmVを下回ったらtrueを返すPredicate。
// 空転させて電池調節するときに使える。
//
// 実方
class BatteryPredicate : public Predicate
{
private:
    int targetVoltage;

public:
    BatteryPredicate(int targetVoltage);
    virtual ~BatteryPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual BatteryPredicate *generateReversePredicate() override;
};

#endif