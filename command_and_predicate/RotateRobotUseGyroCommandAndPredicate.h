#ifndef RotateRobotUseGyroCommandAndPredicate_H
#define RotateRobotUseGyroCommandAndPredicate_H

#include "CommandAndPredicate.h"

// RotateRobotUseGyroCommandAndPredicate
// ロボットを指定角度旋回させるためのCommandとPredicate。
// 
// 実方
class RotateRobotUseGyroCommandAndPredicate : public CommandAndPredicate
{
private:
public:
    RotateRobotUseGyroCommandAndPredicate(int targetAngle, int pwm, RobotAPI *robotAPI);
    virtual ~RotateRobotUseGyroCommandAndPredicate();
};


#endif