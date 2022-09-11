#ifndef FacingAngle_H
#define FacingAngle_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Walker.h"

enum FacingAngleMode
{
    FA_Gyro,
    FA_WheelCount,
};

// 渡された角度を向くまで旋回し続けるコマンド。
// スラローム進入後の段差での角度ずれを解決するためのもの。
// 指定角度旋回したいのであれば、RotateRobotUseGyroCommandAndPredicateを使ってください
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
    FacingAngle(FacingAngleMode mode, int pwm, int targetAngle);
    ~FacingAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Command *generateReverseCommand() override;
    virtual bool isFinished() override;
};
#endif