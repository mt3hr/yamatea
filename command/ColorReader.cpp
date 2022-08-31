#include "ColorReader.h"
#include "RobotAPI.h"
#include "Sensor.h"

ColorReader::ColorReader(){};

ColorReader::~ColorReader(){};

void ColorReader::run(RobotAPI *robotAPI)
{
    color = robotAPI->getColorSensor()->getColorNumber();
}

void ColorReader::preparation(RobotAPI *robotAPI)
{
}

Command *ColorReader::generateReverseCommand()
{
    return new ColorReader();
}

colorid_t ColorReader::getColor()
{
    return color;
}