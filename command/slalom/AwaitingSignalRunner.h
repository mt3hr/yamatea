#ifndef AwaitingSignalRunner_H
#define AwaitingSignalRunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

class AwaitingSignalRunner : public ObstacleDetectRunner
{
private:

public:
    AwaitingSignalRunner(ObstacleDetector *obstacleDetector);
    void run(RobotAPI *robotAPI) override;
    AwaitingSignalRunner *generateReverseCommand() override;
};

#endif