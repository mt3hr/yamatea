#include "StraightBetweenRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

StraightBetweenRunner::StraightBetweenRunner(float p, int walkerPow, int rotatePow, WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector) : UFORunner(p, 0, walkerPow, rotatePow, wheelController, sonarSensor, obstacleDetector)
{
    this->p = p;
    this->walkerPow = walkerPow;
    this->rotatePow = rotatePow;
    this->wheelController = wheelController;
    this->sonarSensor = sonarSensor;
}

StraightBetweenRunner *StraightBetweenRunner::generateReverseCommand()
{
    return new StraightBetweenRunner(p, walkerPow, rotatePow, wheelController, sonarSensor, getObstacleDetector()->generateReverseCommand());
}