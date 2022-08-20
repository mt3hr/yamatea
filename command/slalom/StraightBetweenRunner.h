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

    float swingLeftAngle;
    float swingRightAngle;
    int targetLeftDistance;
    int targetRightDistance;

public:
    StraightBetweenRunner(int walkerPow, int rotatePow, float swingLeftAngle, float swingRightAngle, int targetLeftDistance, int targetRightDistance);
    virtual ~StraightBetweenRunner();
    virtual StraightBetweenRunner *generateReverseCommand() override;
};

#endif