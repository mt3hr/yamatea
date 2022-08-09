#ifndef RotateRobot_H
#define RotateRobot_H

#include "WheelController.h"
#include "CommandAndPredicate.h"

// ロボット旋回コマンド生成関数
//
// 実方
CommandAndPredicate *generateRotateRobotCommand(int targetAngle, int pwm, WheelController *wheelController);

#endif