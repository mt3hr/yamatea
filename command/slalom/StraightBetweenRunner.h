#ifndef StraightBetweenRunner_H
#define StraightBetweenRunner_H

#include "UFORunner.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

class StraightBetweenRunner : public UFORunner
{
private:
    float n;
    int walkerPow;
    int rotatePow;
    bool turnToI;
    bool iIsLeft;

public:
    StraightBetweenRunner(int walkerPow, int rotatePow, bool iIsLeft, bool turnToI, ObstacleDetector *obstacleDetector);
    StraightBetweenRunner *generateReverseCommand() override;
};

#endif