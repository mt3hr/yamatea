#include "SetPWMCoefficient.h"
#include "RobotAPI.h"
#include "Setting.h"

SetPWMCoefficient::SetPWMCoefficient()
{
}

SetPWMCoefficient::~SetPWMCoefficient()
{
}

void SetPWMCoefficient::run(RobotAPI *robotAPI)
{
    float coefficient = calcPWMCoefficient();
    leftWheelPWMCorrectedValue = coefficient;
    rightWheelPWMCorrectedValue = coefficient;
}

void SetPWMCoefficient::preparation(RobotAPI *robotAPI)
{
    return;
}

SetPWMCoefficient *SetPWMCoefficient::generateReverseCommand()
{
    return new SetPWMCoefficient();
}

float SetPWMCoefficient::calcPWMCoefficient()
{
    float voltage = ev3_battery_voltage_mV();
    float coefficient = 100 / (75.69 + 3.1006 * voltage / 1000);
    return coefficient;
}
