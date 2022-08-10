#ifndef SteeringRobot_H
#define SteeringRobot_H

#include "Command.h"
#include "Steering.h"
#include "Motor.h"

using namespace ev3api;

// ev3api::Steeringクラスを使ったらいい感じに動くのかなと思ったけど、そんなことはなかった。
// Walkerでいいじゃん。
// Walker使います。
class SteeringRobot : public Command
{
private:
    int pwm;
    int angle;
    Steering *steering;
    Motor *leftWheel;
    Motor *rightWheel;
    bool calledSteering = false;

public:
    SteeringRobot(int pwm, int angle, Motor *leftWheel, Motor *rightWheel);
    ~SteeringRobot();
    void run();
    SteeringRobot *generateReverseCommand();
};

#endif