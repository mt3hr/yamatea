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