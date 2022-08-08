#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"

ObstacleDetectRunner::ObstacleDetectRunner(ObstacleDetector *od)
{
    obstacleDetector = od;
}

ObstacleDetectRunner::~ObstacleDetectRunner()
{
    delete obstacleDetector;
}

ObstacleDetector *ObstacleDetectRunner::getObstacleDetector()
{
    return obstacleDetector;
}

ObstacleDetectRunner *ObstacleDetectRunner::generateReverseCommand() {
    return new ObstacleDetectRunner(obstacleDetector);
}