#ifndef StraightBetweenRunner_H
#define StraightBetweenRunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

class StraightBetweenRunner : public ObstacleDetectRunner
{
private:
    WheelController *wheelController;
    SonarSensor *sonarSensor;

public:
    StraightBetweenRunner(WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    void run() override;
    StraightBetweenRunner *generateReverseCommand() override;
};

#endif