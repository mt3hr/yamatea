#ifndef ResetArmAngle_H
#define ResetArmAngle_H

#include "Command.h"
#include "Predicate.h"
#include "FinishConfirmable.h"
#include "RobotAPI.h"
#include "Setting.h"
#include "ArmController.h"
#include "NumberOfTimesPredicate.h"
#include "MotorRotateAnglePredicate.h"
#include "Stopper.h"

using namespace ev3api;

// ResetArmAngleState
// ResetArmAngleの状態
//
// 実方
enum ResetArmAngleState
{
    RAAS_PullingArm,
    RAAS_FixingArmAngle,
    RAAS_Finish,
};

// ResetArmAngle
// キャリブレーション前にカラーセンサの乗ったアーム角度を調節するコマンド。
//
// 実方
class ResetArmAngle : public Command, public FinishConfirmable
{
private:
    ResetArmAngleState state;

    Command *pullArm = new ArmController(-pwmForResetArm);
    Predicate *pullArmPredicate = new NumberOfTimesPredicate(numberOfTimesForPullWhenResetArm);
    bool initedPullArm = false;

    Command *fixArm = new ArmController(pwmForResetArm);
    Predicate *fixArmPredicate;
    bool initedFixArm = false;

    Stopper *stopper = new Stopper();

public:
    ResetArmAngle();
    virtual ~ResetArmAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ResetArmAngle *generateReverseCommand() override;
    virtual bool isFinished() override;
};

#endif