#ifndef PIDFacingAngleAbs_H
#define PIDFacingAngleAbs_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "FacingAngleAbs.h"
#include "Walker.h"

// PIDFacingAngleAbs
// 渡された角度を向くまで旋回し続けるコマンド。
// スラローム進入後の段差での角度ずれを解決するためのもの。
// 指定角度旋回したいのであれば、RotateRobotUseGyroCommandAndPredicateを使ってください
//
// 実方
class PIDFacingAngleAbs : public Command, public FinishConfirmable
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

    int targetAngle = 0;
    int currentAngle = 0;
    float currentAnglef = 0;
    bool finish = false;

public:
    PIDFacingAngleAbs(FacingAngleMode mode, int angle, float kp, float ki, float kd, float dt);
    ~PIDFacingAngleAbs();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDFacingAngleAbs *generateReverseCommand() override;
    virtual bool isFinished() override;
};
#endif