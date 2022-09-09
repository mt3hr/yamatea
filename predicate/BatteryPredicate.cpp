#include "BatteryPredicate.h"
#include "RobotAPI.h"

using namespace ev3api;

BatteryPredicate::BatteryPredicate(int targetVoltage)
{
    this->targetVoltage = targetVoltage;
}

BatteryPredicate::~BatteryPredicate()
{
}

bool BatteryPredicate::test(RobotAPI *robotAPI)
{
    return ev3_battery_voltage_mV() <= targetVoltage;
}

void BatteryPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

Predicate *BatteryPredicate::generateReversePredicate()
{
    return new BatteryPredicate(targetVoltage);
}