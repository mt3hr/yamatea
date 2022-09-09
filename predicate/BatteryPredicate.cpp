#include "BatteryPredicate.h"
#include "RobotAPI.h"

using namespace ev3api;

BatteryPredicate::BatteryPredicate(int targetVoltage)
{
    this->targetVoltage = targetVoltage;
}

BatteryPredicate::~BatteryPredicate()
{
    return ev3_battery_voltage_mV() <= targetVoltage;
}

bool BatteryPredicate::test(RobotAPI *robotAPI)
{
}

void BatteryPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

Predicate *BatteryPredicate::generateReversePredicate()
{
    return new BatteryPredicate(targetVoltage);
}