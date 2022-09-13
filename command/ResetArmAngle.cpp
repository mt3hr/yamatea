#include "ResetArmAngle.h"
#include "Command.h"
#include "Predicate.h"
#include "FinishConfirmable.h"
#include "RobotAPI.h"
#include "Setting.h"
#include "ArmController.h"
#include "NumberOfTimesPredicate.h"
#include "MotorRotateAnglePredicate.h"
#include "Stopper.h"

ResetArmAngle::ResetArmAngle()
{
    this->state = RAAS_PullingArm;
};

ResetArmAngle::~ResetArmAngle()
{
    delete pullArm;
    delete pullArmPredicate;
    delete fixArm;
    delete fixArmPredicate;
    delete stopper;
};

void ResetArmAngle::run(RobotAPI *robotAPI)
{
    switch (state)
    {
    case RAAS_PullingArm:
    {
        if (!initedPullArm)
        {
            initedPullArm = true;
            pullArm->preparation(robotAPI);
            pullArmPredicate->preparation(robotAPI);
            return;
        }
        if (!pullArmPredicate->test(robotAPI))
        {
            pullArm->run(robotAPI);
        }
        else
        {
            state = RAAS_FixingArmAngle;
        }
        break;
    }
    case RAAS_FixingArmAngle:
    {
        if (!initedFixArm)
        {
            initedFixArm = true;
            fixArm->preparation(robotAPI);
            fixArmPredicate->preparation(robotAPI);
            return;
        }
        if (!fixArmPredicate->test(robotAPI))
        {
            fixArm->run(robotAPI);
        }
        else
        {
            stopper->run(robotAPI);
            state = RAAS_Finish;
        }
        break;
    }

    default:
        break;
    }
}

void ResetArmAngle::preparation(RobotAPI *robotAPI)
{
    fixArmPredicate = new MotorRotateAnglePredicate(angleForResetArm, robotAPI->getArmMotor());
    return;
}

ResetArmAngle *ResetArmAngle::generateReverseCommand()
{
    return new ResetArmAngle();
}

bool ResetArmAngle::isFinished()
{
    return state == RAAS_Finish;
}