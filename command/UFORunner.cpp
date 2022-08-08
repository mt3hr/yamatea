#include "UFORunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

UFORunner::UFORunner(WheelController *wc, SonarSensor *ss, ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
    wheelController = wc;
    sonarSensor = ss;
}

void UFORunner::run()
{
    // TODO
    return;
}

UFORunner *UFORunner::generateReverseCommand()
{
    return new UFORunner(wheelController, sonarSensor, getObstacleDetector()->generateReverseCommand());
}