#include "ObstacleDetector.h"

ObstacleDetector::~ObstacleDetector()
{
}

int ObstacleDetector::getLeftObstacleDistance()
{
    return 0;
}


int ObstacleDetector::getRightObstacleDistance()
{
    return 0;
}

bool ObstacleDetector::isDetectedLeftObstacleDistance()
{
    return false;
}

bool ObstacleDetector::isDetectedRightObstacleDistance()
{
    return false;
}

ObstacleDetector *ObstacleDetector::generateReverseCommand()
{
    return new ObstacleDetector();
}