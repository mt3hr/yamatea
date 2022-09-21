#ifndef FacingAngle_H
#define FacingAngle_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Walker.h"
#include "FacingAngleAbs.h"

// 渡された角度を向くまで旋回し続けるコマンド。
// スラローム進入後の段差での角度ずれを解決するためのもの。
// 指定角度旋回したいのであれば、RotateRobotUseGyroCommandAndPredicateを使ってください
//
// 実方
class FacingAngle : public Command, public FinishConfirmable
{
private:
    FacingAngleMode mode;
    int pwm;
    int targetAngle;
    int angle;
    bool finish = false;

    Walker *turnLeft;
    Walker *turnRight;

public:
    FacingAngle(FacingAngleMode mode, int pwm, int angle);
    ~FacingAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual FacingAngle *generateReverseCommand() override;
    virtual bool isFinished() override;
};
#endif