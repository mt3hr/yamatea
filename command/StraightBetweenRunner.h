#ifndef StraightBetweenRunner_H
#define StraightBetweenRunner_H

#include "UFORunner.h"
#include "SonarSensor.h"
#include "WheelController.h"

class StraightBetweenRunner : public UFORunner
{
private:
    float n;
    int walkerPow;
    int rotatePow;
    WheelController *wheelController;
    SonarSensor *sonarSensor;

public:
    StraightBetweenRunner(int walkerPow, int rotatePow, WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    StraightBetweenRunner *generateReverseCommand() override;
};

#endif