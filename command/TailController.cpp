#include "TailController.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

TailController::TailController(int p)
{
    pwm = p;
}

TailController::~TailController()
{
}

void TailController::run(RobotAPI *robotAPI)
{
    robotAPI->getTailMotor()->setPWM(pwm);
}

void TailController::preparation(RobotAPI *robotAPI)
{
    return;
}

TailController *TailController::generateReverseCommand()
{
    return new TailController(pwm);
}