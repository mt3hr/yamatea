#ifndef FacingAngle_H
#define FacingAngle_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Walker.h"

// 渡された角度を向くまで旋回し続けるコマンド。
// スラローム進入後の段差での角度ずれを解決するためのもの。
// 指定角度旋回したいのであれば、RotateRobotUseGyroCommandAndPredicateを使ってください
class FacingAngle : public Command, public FinishConfirmable
{
private:
    int pwm;
    int targetAngle;
    int angle;
    bool finish = false;
    bool useGyro = false;

    Walker *turnLeft;
    Walker *turnRight;

public:
    FacingAngle(int pwm, int targetAngle, bool useGyro);
    ~FacingAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Command *generateReverseCommand() override;
    virtual bool isFinished() override;
};
#endif