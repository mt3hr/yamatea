#include "ObstacleDetectRunner.h"

ObstacleDetectRunner::ObstacleDetectRunner()
{
    return;
}

void ObstacleDetectRunner::setLeftObstacleDistance(int lod)
{
    leftObstacleDistance = lod;
    return;
}

void ObstacleDetectRunner::setRightObstacleDistance(int rod)
{
    rightObstacleDistance = rod;
    return;
}

int ObstacleDetectRunner::getLeftObstacleDistance()
{
    return leftObstacleDistance;
}

int ObstacleDetectRunner::getRightObstacleDistance()
{
    return rightObstacleDistance;
}
