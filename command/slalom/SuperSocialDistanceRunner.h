#ifndef SuperSocialDistanceRunner_H
#define SuperSocialDistanceRunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

class SuperSocialDistanceRunner : public ObstacleDetectRunner
{
private:
public:
    SuperSocialDistanceRunner(ObstacleDetector *obstacleDetector);
    virtual ~SuperSocialDistanceRunner();
    virtual void run(RobotAPI *robotAPI) override;
    virtual SuperSocialDistanceRunner *generateReverseCommand() override;
};

#endif