#ifndef UFORunner_H
#define UFORunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

class UFORunner : public ObstacleDetectRunner
{
private:
    WheelController *wheelController;
    SonarSensor *sonarSensor;

public:
    UFORunner(WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    void run() override;
    UFORunner *generateReverseCommand() override;
};

#endif