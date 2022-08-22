#ifndef StraightBetweenRunner_H
#define StraightBetweenRunner_H

#include "UFORunner.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

// StraightBetweenRunner 
// 直進行軍走行するクラス。
// 要求モデル参照。
//
// 実方
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
    StraightBetweenRunner(int walkerPow, int rotatePow);
    virtual ~StraightBetweenRunner();
    virtual StraightBetweenRunner *generateReverseCommand() override;
};

#endif