#ifndef AwaitingSignalRunner_H
#define AwaitingSignalRunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

// TODO 未実装
// AwaitingSignalRunner
// 指示待ち走行するクラス
// 要求モデル参照
//
// 実方
class AwaitingSignalRunner : public ObstacleDetectRunner
{
private:
public:
    AwaitingSignalRunner(ObstacleDetector *obstacleDetector);
    ~AwaitingSignalRunner();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual AwaitingSignalRunner *generateReverseCommand() override;
};

#endif