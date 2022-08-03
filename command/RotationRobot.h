#ifndef RotationRobot_H
#define RotationRobot_H

#include "WheelController.h"

// ロボット旋回コマンド生成関数
//
// 実方
CommandAndPredicate *generateRotationRobotCommand(int targetAngle, WheelController *wheelController);

#endif