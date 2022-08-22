#include "StraightBetweenRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "UFORunner.h"
#include "RobotAPI.h"

StraightBetweenRunner::StraightBetweenRunner(int walkerPow, int rotatePow) : UFORunner(0.01, walkerPow, rotatePow)
{
    this->walkerPow = walkerPow;
    this->rotatePow = rotatePow;
    this->swingLeftAngle = swingLeftAngle;
    this->swingRightAngle = swingRightAngle;
    this->targetLeftDistance = targetLeftDistance;
    this->targetRightDistance = targetRightDistance;
};

StraightBetweenRunner::~StraightBetweenRunner()
{
}

StraightBetweenRunner *StraightBetweenRunner::generateReverseCommand()
{
    return new StraightBetweenRunner(walkerPow, rotatePow);
}