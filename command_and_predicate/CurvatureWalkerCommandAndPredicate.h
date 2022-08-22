#ifndef CurvatureWalker_H
#define CurvatureWalker_H

#include "CommandAndPredicate.h"
#include "RobotAPI.h"

// CurvatureWalkerCommandAndPredicate 
// 渡した半径と角度からなる孤をなぞるように進むCommandとPredicate。
// 
// 実方
class CurvatureWalkerCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    CurvatureWalkerCommandAndPredicate(int pwm, float r, float theta, RobotAPI *robotAPI);
    virtual ~CurvatureWalkerCommandAndPredicate();
};

#endif