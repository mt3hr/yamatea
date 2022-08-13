#ifndef RotateRobot_H
#define RotateRobot_H

#include "CommandAndPredicate.h"

// ロボット旋回コマンド生成関数
//
// 実方
CommandAndPredicate *generateRotateRobotCommand(int targetAngle, int pwm, RobotAPI *robotAPI);

#endif