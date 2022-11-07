#include "ResetPWMCoefficient.h"
#include "RobotAPI.h"

ResetPWMCoefficient::ResetPWMCoefficient(/* args */)
{
}

ResetPWMCoefficient::~ResetPWMCoefficient()
{
}

void ResetPWMCoefficient::run(RobotAPI *robotAPI)
{
    SetPWMCoefficient::run(robotAPI);
}

void ResetPWMCoefficient::preparation(RobotAPI *robotAPI)
{
    return;
}

ResetPWMCoefficient *ResetPWMCoefficient::generateReverseCommand()
{
    return new ResetPWMCoefficient();
}