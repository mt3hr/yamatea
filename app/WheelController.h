#ifndef WheelController_H
#define WheelController_H

#include "Motor.h"

using namespace ev3api;

// WheelController
// 左右車輪を管理する。
// モデルをきれいにするためのクラス
//
// 実方
class WheelController
{
private:
    Motor *leftWheel;
    Motor *rightWheel;

public:
    WheelController(Motor *leftWheel, Motor *rightWheel);
    Motor *getLeftWheel();
    Motor *getRightWheel();
    void setLeftWheelPWM(int pwm);
    void setRightWheelPWM(int pwm);
};

#endif