#ifndef SuperSocialDistanceRunner_H
#define SuperSocialDistanceRunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

class SuperSocialDistanceRunner : public ObstacleDetectRunner
{
private:
    WheelController *wheelController;
    SonarSensor *sonarSensor;

public:
    SuperSocialDistanceRunner(WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    void run() override;
    SuperSocialDistanceRunner *generateReverseCommand() override;
};

#endif