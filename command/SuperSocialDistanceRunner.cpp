#include "SuperSocialDistanceRunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

SuperSocialDistanceRunner::SuperSocialDistanceRunner(WheelController *wc, SonarSensor *ss,ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
    wheelController = wc;
    sonarSensor = ss;
}

void SuperSocialDistanceRunner::run()
{
    // TODO
    return;
}

SuperSocialDistanceRunner *SuperSocialDistanceRunner::generateReverseCommand()
{
    return new SuperSocialDistanceRunner(wheelController, sonarSensor, getObstacleDetector()->generateReverseCommand());
}