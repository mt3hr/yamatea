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

float ObstacleDetector::getLeftObstacleAngle()
{
    return 0;
}

float ObstacleDetector::getRightObstacleAngle()
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

bool ObstacleDetector::isDetectedLeftObstacleAngle()
{
    return false;
}

bool ObstacleDetector::isDetectedRightObstacleAngle()
{
    return false;
}