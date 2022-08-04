#ifndef SuperSocialDistanceRunner_H
#define SuperSocialDistanceRunner_H

#include "ObstacleDetectRunner.h"

class SuperSocialDistanceRunner : public ObstacleDetectRunner
{
private:
public:
    SuperSocialDistanceRunner();
    void run() override;
    SuperSocialDistanceRunner *generateReverseCommand() override;
};

#endif