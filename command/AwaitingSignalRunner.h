#ifndef AwaitingSignalRunner_H
#define AwaitingSignalRunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"

class AwaitingSignalRunner : public ObstacleDetectRunner
{
private:
    WheelController *wheelController;
    SonarSensor *sonarSensor;

public:
    AwaitingSignalRunner(WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    void run() override;
    AwaitingSignalRunner *generateReverseCommand() override;
};

#endif