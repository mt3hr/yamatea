#ifndef FacingAngle_H
#define FacingAngle_H

#include "Command.h"
#include "Walker.h"

// 渡された角度を向くまで旋回し続けるコマンド。
// スラローム進入後の段差での角度ずれを解決するためのもの。
// 指定角度旋回したいのであれば、RotateRobotUseGyroCommandAndPredicateを使ってください
class FacingAngle : public Command
{
private:
    int pwm;
    int targetAngle;
    int angle;
    bool finish;

    Walker *turnLeft;
    Walker *turnRight;

public:
    FacingAngle(int pwm, int targetAngle);
    ~FacingAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Command *generateReverseCommand() override;
    virtual bool isFinished() override;
};
#endif