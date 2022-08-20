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
    ~AwaitingSignalRunner();
    virtual void run(RobotAPI *robotAPI) override;
    virtual AwaitingSignalRunner *generateReverseCommand() override;
};

#endif