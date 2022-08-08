#include "StraightBetweenRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

StraightBetweenRunner::StraightBetweenRunner(WheelController *wc, SonarSensor *ss, ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
    wheelController = wc;
    sonarSensor = ss;
}

void StraightBetweenRunner::run()
{
    // TODO
    return;
}

StraightBetweenRunner *StraightBetweenRunner::generateReverseCommand()
{
    // TODO
    return new StraightBetweenRunner(wheelController, sonarSensor, getObstacleDetector()->generateReverseCommand());
}