#ifndef CurvatureWalker_H
#define CurvatureWalker_H

#include "CommandAndPredicate.h"
#include "RobotAPI.h"

enum CWCAPMode
{
    CWCMP_Gyro,
    CWCMP_WheelCount,
};

// CurvatureWalkerCommandAndPredicate
// 渡した半径と角度からなる円の孤をなぞるように進むCommandとPredicate。
//
// 実方
class CurvatureWalkerCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    CurvatureWalkerCommandAndPredicate(CWCAPMode mode, float pwm, float r, float theta, RobotAPI *robotAPI);
    virtual ~CurvatureWalkerCommandAndPredicate();
};

#endif