#ifndef SuperSocialDistanceRunner_H
#define SuperSocialDistanceRunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RobotAPI.h"

// TODO 未実装
// SuperSocialDistanceRunner
// 超ソーシャルディスタンス走行クラス。
// 要求モデル参照。
//
// 実方
class SuperSocialDistanceRunner : public ObstacleDetectRunner
{
private:
public:
    SuperSocialDistanceRunner(ObstacleDetector *obstacleDetector);
    virtual ~SuperSocialDistanceRunner();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual SuperSocialDistanceRunner *generateReverseCommand() override;
};

#endif