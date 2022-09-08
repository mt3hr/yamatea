#include "ResetMeasAngle.h"
#include "RobotAPI.h"

ResetMeasAngle::ResetMeasAngle(){};

ResetMeasAngle::~ResetMeasAngle(){};

void ResetMeasAngle::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->reset();
    robotAPI->getRightWheel()->reset();
    robotAPI->getMeasAngle()->reset();
}

void ResetMeasAngle::preparation(RobotAPI *robotAPI)
{
}

ResetMeasAngle *ResetMeasAngle::generateReverseCommand()
{
    return new ResetMeasAngle();
}