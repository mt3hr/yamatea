#ifndef ObstacleDetectRunner_H
#define ObstacleDetectRunner_H

#include "Command.h"
#include "ObstacleDetector.h"

// ObstacleDetectRunner
// 難所スラロームペットボトル回避走行の基底クラス。
//
// 実方
class ObstacleDetectRunner : public Command
{
private:
    ObstacleDetector *obstacleDetector;

protected:
    virtual ObstacleDetector *getObstacleDetector();
    virtual void setObstacleDetector(ObstacleDetector *obstacleDetector);

public:
    ObstacleDetectRunner(ObstacleDetector *obstacleDetector);
    ObstacleDetectRunner();
    virtual ~ObstacleDetectRunner();
    virtual ObstacleDetectRunner *generateReverseCommand() override;
};

#endif
