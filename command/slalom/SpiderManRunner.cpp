#include "SpiderManRunner.h"
#include "DebugUtil.h"
#include "Walker.h"
#include "FinishConfirmable.h"

SpiderManRunner::SpiderManRunner(int pwm, float angleLeftCenter, float angleCenterRight, int targetLeft, int targetCenter, int targetRight)
{
    this->pwm = pwm;
    this->angleLeftCenter = angleLeftCenter;
    this->angleCenterRight = angleCenterRight;
    this->targetLeft = targetLeft;
    this->targetCenter = targetCenter;
    this->targetRight = targetRight;
};

SpiderManRunner::~SpiderManRunner(){
    // TODO not implements
};

void SpiderManRunner::run(RobotAPI *robotAPI)
{
    return;
}

void SpiderManRunner::preparation(RobotAPI *robotAPI)
{
    return;
}

SpiderManRunner *SpiderManRunner::generateReverseCommand()
{
    // TODO not implements
    // return new SpiderManRunner();
    return new SpiderManRunner(pwm, angleLeftCenter, angleCenterRight, targetLeft, targetCenter, targetRight);
}

bool SpiderManRunner::isFinished()
{
    return state == SPDM_FINISH;
}