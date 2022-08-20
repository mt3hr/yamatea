#ifndef CurvatureWalker_H
#define CurvatureWalker_H

#include "CommandAndPredicate.h"
#include "RobotAPI.h"

class CurvatureWalkerCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    CurvatureWalkerCommandAndPredicate(int pwm, float r, float theta, RobotAPI *robotAPI);
    virtual ~CurvatureWalkerCommandAndPredicate();
};

#endif