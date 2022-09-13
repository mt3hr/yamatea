#include "ColorReader.h"
#include "RobotAPI.h"
#include "Sensor.h"
#include "DebugUtil.h"

ColorReader::ColorReader()
{
    color = new colorid_t;
};

ColorReader::~ColorReader(){};

void ColorReader::run(RobotAPI *robotAPI)
{
    *color = robotAPI->getColorSensor()->getColorNumber();
    writeDebug("ColorReader: ");
    writeEndLineDebug();
    writeDebug("color id: ");
    writeDebug(((int)color));
    flushDebug(DEBUG, robotAPI);
}

void ColorReader::preparation(RobotAPI *robotAPI)
{
}

ColorReader *ColorReader::generateReverseCommand()
{
    return new ColorReader();
}

colorid_t *ColorReader::getColorPtr()
{
    return color;
}