#ifndef StraightBetweenRunner_H
#define StraightBetweenRunner_H

#include "UFORunner.h"
#include "SonarSensor.h"
#include "WheelController.h"

class StraightBetweenRunner : public UFORunner
{
private:
    float p;
    int walkerPow;
    int rotatePow;
    WheelController *wheelController;
    SonarSensor *sonarSensor;

public:
    StraightBetweenRunner(float p, int walkerPow, int rotatePow, WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    StraightBetweenRunner *generateReverseCommand() override;
};

#endif