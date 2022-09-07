#include "ResetGyroSensor.h"
#include "Command.h"
#include "RobotAPI.h"

ResetGyroSensor::ResetGyroSensor(){};

ResetGyroSensor::~ResetGyroSensor(){};

void ResetGyroSensor::run(RobotAPI *robotAPI)
{
    robotAPI->getGyroSensor()->reset();
}

void ResetGyroSensor::preparation(RobotAPI *robotAPI)
{
}

ResetGyroSensor *ResetGyroSensor::generateReverseCommand()
{
    return new ResetGyroSensor();
}