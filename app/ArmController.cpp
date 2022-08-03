#include "ArmController.h"
#include "Motor.h"

using namespace ev3api;

ArmController::ArmController(int p, Motor *am)
{
    pwm = p;
    armMotor = am;
}

void ArmController::run()
{
    armMotor->setPWM(pwm);
}

Command *ArmController::generateReverseCommand()
{
    return new ArmController(-pwm, armMotor);
}