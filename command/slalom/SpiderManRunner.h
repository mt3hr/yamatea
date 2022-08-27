#ifndef SpiderManRunner_H
#define SpiderManRunner_H

#include "ObstacleDetectRunner.h"
#include "FinishConfirmable.h"

class SpiderManRunner : public ObstacleDetectRunner, public FinishConfirmable
{
private:
public:
    SpiderManRunner();
    virtual ~SpiderManRunner();
    virtual void run(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual SpiderManRunner *generateReverseCommand();
    virtual bool isFinished();
};

#endif