#ifndef PIDFacingAngle_H
#define PIDFacingAngle_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Walker.h"
#include "FacingAngleAbs.h"

// PIDFacingAngle
// 渡された角度を向くまで旋回し続けるコマンド。
// スラローム進入後の段差での角度ずれを解決するためのもの。
// 指定角度旋回したいのであれば、RotateRobotUseGyroCommandAndPredicateを使ってください
//
// 実方
class PIDFacingAngle : public Command, public FinishConfirmable
{
private:
    FacingAngleMode mode;

    float kp;
    float ki;
    float kd;
    float dt;

    float p;
    float i;
    float d;
    float pid;
    float beforeP = 0;
    float integral = 0;
    float rightPower;
    float leftPower;

    int targetAngle;
    int angle;
    bool finish = false;

public:
    PIDFacingAngle(FacingAngleMode mode, int angle, float kp, float ki, float kd, float dt);
    ~PIDFacingAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDFacingAngle *generateReverseCommand() override;
    virtual bool isFinished() override;
};
#endif