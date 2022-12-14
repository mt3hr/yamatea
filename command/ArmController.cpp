#include "ArmController.h"
#include "Motor.h"
#include "RobotAPI.h"

using namespace ev3api;

ArmController::ArmController(int p)
{
    pwm = p;
}

ArmController::~ArmController()
{
}

void ArmController::run(RobotAPI *robotAPI)
{
    robotAPI->getArmMotor()->setPWM(pwm);
}

void ArmController::preparation(RobotAPI *robotAPI)
{
    return;
}

ArmController *ArmController::generateReverseCommand()
{
    return new ArmController(pwm);
}