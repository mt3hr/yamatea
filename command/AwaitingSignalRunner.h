#ifndef AwaitingSignalRunner_H
#define AwaitingSignalRunner_H

#include "ObstacleDetectRunner.h"

class AwaitingSignalRunner : public ObstacleDetectRunner
{
private:
public:
    AwaitingSignalRunner();
    void run() override;
    AwaitingSignalRunner *generateReverseCommand() override;
};

#endif