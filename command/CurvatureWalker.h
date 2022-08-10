#ifndef CurvatureWalker_H
#define CurvatureWalker_H

#include "CommandAndPredicate.h"
#include "WheelController.h"

CommandAndPredicate *generateCurvatureWalkerWithTheta(int pwm, float r, float theta, bool clock, WheelController *wheelController);

#endif