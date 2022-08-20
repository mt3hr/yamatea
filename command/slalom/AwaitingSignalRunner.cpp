#include "AwaitingSignalRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

AwaitingSignalRunner::AwaitingSignalRunner(ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
}

AwaitingSignalRunner::~AwaitingSignalRunner()
{
}

void AwaitingSignalRunner::run(RobotAPI *robotAPI)
{
    // TODO
    return;
}

AwaitingSignalRunner *AwaitingSignalRunner::generateReverseCommand()
{
    return new AwaitingSignalRunner(getObstacleDetector()->generateReverseCommand());
}