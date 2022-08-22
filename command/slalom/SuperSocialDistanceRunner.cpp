#include "SuperSocialDistanceRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

SuperSocialDistanceRunner::SuperSocialDistanceRunner(ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
}

SuperSocialDistanceRunner::~SuperSocialDistanceRunner()
{
}

void SuperSocialDistanceRunner::run(RobotAPI *robotAPI)
{
    // TODO
    return;
}

void SuperSocialDistanceRunner::preparation(RobotAPI *robotAPI)
{
    return;
}

SuperSocialDistanceRunner *SuperSocialDistanceRunner::generateReverseCommand()
{
    return new SuperSocialDistanceRunner(getObstacleDetector()->generateReverseCommand());
}