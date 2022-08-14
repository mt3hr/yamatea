#include "StraightBetweenRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "UFORunner.h"
#include "RobotAPI.h"

StraightBetweenRunner::StraightBetweenRunner(int walkerPow, int rotatePow, ObstacleDetector *obstacleDetector) : UFORunner(0, walkerPow, rotatePow, obstacleDetector)
{
    this->walkerPow = walkerPow;
    this->rotatePow = rotatePow;
};

StraightBetweenRunner *StraightBetweenRunner::generateReverseCommand()
{
    return new StraightBetweenRunner(walkerPow, rotatePow, getObstacleDetector()->generateReverseCommand());
}