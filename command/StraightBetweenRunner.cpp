#include "StraightBetweenRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"
#include "UFORunner.h"

StraightBetweenRunner::StraightBetweenRunner(int walkerPow, int rotatePow, WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector) : UFORunner(0, walkerPow, rotatePow, wheelController, sonarSensor, obstacleDetector)
{
    this->walkerPow = walkerPow;
    this->rotatePow = rotatePow;
    this->wheelController = wheelController;
    this->sonarSensor = sonarSensor;
};

StraightBetweenRunner *StraightBetweenRunner::generateReverseCommand()
{
    return new StraightBetweenRunner(walkerPow, rotatePow, wheelController, sonarSensor, getObstacleDetector()->generateReverseCommand());
}