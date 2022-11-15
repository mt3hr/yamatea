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

    int ambient = robotAPI->getColorSensor()->getAmbient();
    int brightness = robotAPI->getColorSensor()->getBrightness();
    int colorNumber = robotAPI->getColorSensor()->getColorNumber();
    rgb_raw_t rawcolor;
    robotAPI->getColorSensor()->getRawColor(rawcolor);
    int distance = robotAPI->getSonarSensor()->getDistance();

    stringstream ss;
    vector<string> msg;
    ss << batteryMV << "mv";

    msg.push_back(ss.str());
    PrintMessage(msg, true);
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