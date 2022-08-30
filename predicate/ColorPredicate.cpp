#include "ColorPredicate.h"
#include "Sensor.h"

ColorPredicate::ColorPredicate(colorid_t colorID)
{
    this->colorID = colorID;
};

ColorPredicate::~ColorPredicate(){

};

bool ColorPredicate::test(RobotAPI *robotAPI)
{
    return robotAPI->getColorSensor()->getColorNumber() == colorID;
}

void ColorPredicate::preparation(RobotAPI *robotAPI)
{
}

Predicate *ColorPredicate::generateReversePredicate()
{
    return new ColorPredicate(colorID);
}
