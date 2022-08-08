#include "SwingSonarObstacleDetector.h"

SwingSonarObstacleDetector::SwingSonarObstacleDetector(SwingOrder so)
{
    swingOrder = so;
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector(){};

void SwingSonarObstacleDetector::run()
{
    // TODO
}

SwingSonarObstacleDetector *SwingSonarObstacleDetector::generateReverseCommand()
{
    return new SwingSonarObstacleDetector(swingOrder);
}

int SwingSonarObstacleDetector::getLeftObstacleDistance()
{
    return leftObstacleDistance;
}

int SwingSonarObstacleDetector::getRightObstacleDistance()
{
    return rightObstacleDistance;
}

bool SwingSonarObstacleDetector::isDetectedLeftObstacleDistance()
{
    return detectedLeftObstacleDistance;
}

bool SwingSonarObstacleDetector::isDetectedRightObstacleDistance()
{
    return detectedRightObstacleDistance;
}
