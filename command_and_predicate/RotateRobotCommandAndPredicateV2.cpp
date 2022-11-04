#include "RotateRobotCommandAndPredicateV2.h"
#include "Command.h"
#include "Predicate.h"
#include "CommandAndPredicate.h"
#include "Walker.h"
#include "MotorRotateAnglePredicate.h"
#include "FacingRobotUseWheelPredicate.h"
#include "Setting.h"

RotateRobotCommandAndPredicateV2::RotateRobotCommandAndPredicateV2(int targetAngle, float pwm, RobotAPI *robotAPI)
{
    Command *command;
    Predicate *predicate;

    if (targetAngle > 0)
    {
        command = new Walker(pwm, -pwm); // 右に向く
    }
    else
    {
        command = new Walker(-pwm, pwm); // 左に向く
    }
    predicate = new FacingRobotUseWheelPredicate(targetAngle);

    setCommand(command);
    setPredicate(predicate);
}

RotateRobotCommandAndPredicateV2::~RotateRobotCommandAndPredicateV2()
{
}