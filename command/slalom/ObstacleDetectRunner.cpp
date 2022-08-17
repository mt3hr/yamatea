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

void ObstacleDetectRunner::setObstacleDetector(ObstacleDetector *obstacleDetector)
{
    this->obstacleDetector = obstacleDetector;
}

ObstacleDetectRunner *ObstacleDetectRunner::generateReverseCommand()
{
    return new ObstacleDetectRunner(obstacleDetector);
}