#ifndef RotationRobot_H
#define RotationRobot_H

#include "WheelController.h"
#include "CommandAndPredicate.h"

// ロボット旋回コマンド生成関数
//
// 実方
CommandAndPredicate *generateRotationRobotCommand(int targetAngle, int pwm, WheelController *wheelController);

#endif