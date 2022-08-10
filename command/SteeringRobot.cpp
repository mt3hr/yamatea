#include "SteeringRobot.h"
#include "Steering.h"

using namespace ev3api;

SteeringRobot::SteeringRobot(int pwm, int angle, Motor *leftWheel, Motor *rightWheel)
{
    this->pwm = pwm;
    this->angle = angle;
    this->leftWheel = leftWheel;
    this->rightWheel = rightWheel;
    this->steering = new Steering((*leftWheel), (*rightWheel));
}

SteeringRobot::~SteeringRobot()
{
    delete steering;
}

void SteeringRobot::run()
{
    if (!calledSteering)
    {
        calledSteering = true;
        steering->setPower(pwm, angle);
    }
}

SteeringRobot *SteeringRobot::generateReverseCommand()
{
    return new SteeringRobot(pwm, angle, rightWheel, leftWheel);
}