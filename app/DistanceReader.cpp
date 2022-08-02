#include "DistanceReader.h"
#include "Command.h"
#include "SonarSensor.h"
#include "util.h"

using namespace ev3api;

DistanceReader::DistanceReader(SonarSensor *ss)
{
    sonarSensor = ss;
}

void DistanceReader::run()
{
    distanceValue = sonarSensor->getDistance();
    char dStr[20];

    sprintf(dStr, "distance:%d", distanceValue);
    msg_f("distance reader", 1);
    msg_f(dStr, 2);
    msg_f("", 3);
    msg_f("", 4);
    msg_f("", 5);
    msg_f("", 6);
    msg_f("", 7);
}

Command *DistanceReader::generateReverseCommand()
{
    return new DistanceReader(sonarSensor);
}
