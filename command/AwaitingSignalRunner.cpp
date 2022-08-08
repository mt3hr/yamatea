#include "AwaitingSignalRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

AwaitingSignalRunner::AwaitingSignalRunner(WheelController *wc, SonarSensor *ss, ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
    wheelController = wc;
    sonarSensor = ss;
}

void AwaitingSignalRunner::run()
{
    // TODO
    return;
}

AwaitingSignalRunner *AwaitingSignalRunner::generateReverseCommand()
{
    // TODO
    return new AwaitingSignalRunner(wheelController, sonarSensor, getObstacleDetector()->generateReverseCommand());
}