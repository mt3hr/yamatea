#include "StraightBetweenRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "UFORunner.h"
#include "RobotAPI.h"

StraightBetweenRunner::StraightBetweenRunner(int walkerPow, int rotatePow, bool iIsLeft, bool turnToI, ObstacleDetector *obstacleDetector) : UFORunner(0.01, walkerPow, rotatePow, iIsLeft, turnToI, obstacleDetector)
{
    this->iIsLeft = iIsLeft;
    this->turnToI = turnToI;
    this->walkerPow = walkerPow;
    this->rotatePow = rotatePow;
};

StraightBetweenRunner *StraightBetweenRunner::generateReverseCommand()
{
    return new StraightBetweenRunner(walkerPow, rotatePow, iIsLeft, turnToI, getObstacleDetector()->generateReverseCommand());
}