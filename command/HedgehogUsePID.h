#ifndef HedgehogUsePID_H
#define HedgehogUsePID_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Stopper.h"
#include "RobotAPI.h"
#include "PIDStraightWalker.h"

enum HedgehogUsePIDState
{
    HHUPS_INIT,
    HHUPS_WALK,
    HHUPS_BACK,
};

// Hedgehog
// ソナーセンサで取得できる値が指定距離になるまで直進、後退を繰り返すコマンド。
//
// 実方
class HedgehogUsePID : public Command,
    public FinishConfirmable
{
private:
    int distance;
    int targetDistance;
    Stopper *stopper;

    PIDStraightWalker *walker;
    PIDStraightWalker *backer;

    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;
    HedgehogUsePIDState state;

public:
    HedgehogUsePID(int targetDistance, float pwm, float straightKp, float straightKi, float straightKd, float straightDt);
    virtual ~HedgehogUsePID();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual HedgehogUsePID *generateReverseCommand() override;
    virtual bool isFinished() override;
};

#endif