#ifndef UFORunner_H
#define UFORunner_H

#include "ObstacleDetectRunner.h"

class UFORunner : public ObstacleDetectRunner
{
private:
public:
    UFORunner();
    void run() override;
    UFORunner *generateReverseCommand() override;
};

#endif