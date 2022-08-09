#include "SwingSonarObstacleDetector.h"
#include "SonarSensor.h"

using namespace ev3api;

SwingSonarObstacleDetector::SwingSonarObstacleDetector(SwingOrder so, SonarSensor *ss)
{
    swingOrder = so;
    sonarSensor = ss;
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector(){};

void SwingSonarObstacleDetector::run()
{
    switch (swingOrder)
    {
    case CENTER_LEFT_RIGHT:
    {
        // TODO
        break;
    }
    case CENTER_RIGHT_LEFT:
    {
        // TODO
        break;
    }
    case LEFT_RIGHT:
    {
        // TODO
        break;
    }
    case RIGHT_LEFT:
    {
        // TODO
        break;
    }

    default:
        break;
    }
}

SwingSonarObstacleDetector *SwingSonarObstacleDetector::generateReverseCommand()
{
    switch (swingOrder)
    {
    case CENTER_LEFT_RIGHT:
        return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT, sonarSensor);
    case CENTER_RIGHT_LEFT:
        return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, sonarSensor);
    case LEFT_RIGHT:
        return new SwingSonarObstacleDetector(RIGHT_LEFT, sonarSensor);
    case RIGHT_LEFT:
        return new SwingSonarObstacleDetector(LEFT_RIGHT, sonarSensor);
    }
    return new SwingSonarObstacleDetector(swingOrder, sonarSensor); // おかしい
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
