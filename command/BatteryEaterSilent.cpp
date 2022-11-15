#include "BatteryEaterSilent.h"
#include "BatteryEaterSilent.h"
#include "RobotAPI.h"
#include "PrintMessage.h"
#include "vector"
#include "string"
#include "sstream"

using namespace ev3api;

BatteryEaterSilent::~BatteryEaterSilent()
{
}

void BatteryEaterSilent::run(RobotAPI *robotAPI)
{
    int batteryMV = ev3_battery_voltage_mV();

    stringstream bs;
    stringstream ts;
    vector<string> msg;
    bs << batteryMV << "mv";
    ts << long(robotAPI->getClock()->now());

    msg.push_back(bs.str());
    msg.push_back(ts.str());
    PrintMessage(msg, true).run(robotAPI);

    rgb_raw_t rawcolor;
    robotAPI->getColorSensor()->getColorNumber();
    robotAPI->getColorSensor()->getRawColor(rawcolor);
    robotAPI->getSonarSensor()->getDistance();
    return;
}

void BatteryEaterSilent::preparation(RobotAPI *robotAPI)
{
    return;
}

BatteryEaterSilent *BatteryEaterSilent::generateReverseCommand()
{
    return new BatteryEaterSilent();
}