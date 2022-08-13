#ifndef CurvatureWalker_H
#define CurvatureWalker_H

#include "CommandAndPredicate.h"
#include "RobotAPI.h"

CommandAndPredicate *generateCurvatureWalkerWithTheta(int pwm, float r, float theta, bool clock, RobotAPI *robotAPI);

#endif