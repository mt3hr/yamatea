#include "SonarDistancePredicate.h"
#include "RobotAPI.h"

SonarDistancePredicate::SonarDistancePredicate(int distance, bool lessThan)
{
    this->distance = distance;
    this->lessThan = lessThan;
};

SonarDistancePredicate::~SonarDistancePredicate()
{

};

bool SonarDistancePredicate::test(RobotAPI *robotAPI)
{
    if (lessThan)
    {
        return robotAPI->getSonarSensor()->getDistance() <= distance;
    }
    else
    {
        return robotAPI->getSonarSensor()->getDistance() >= distance;
    }
}

void SonarDistancePredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

Predicate *SonarDistancePredicate::generateReversePredicate()
{
    return new SonarDistancePredicate(distance, lessThan);
}