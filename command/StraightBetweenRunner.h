#ifndef StraightBetweenRunner_H
#define StraightBetweenRunner_H

#include "ObstacleDetectRunner.h"

class StraightBetweenRunner : public Command
{
private:
public:
    StraightBetweenRunner();
    void run() override;
    StraightBetweenRunner *generateReverseCommand() override;
};

#endif