#include "RobotAPI.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"

using namespace ev3api;

RobotAPI::RobotAPI(TouchSensor *touchSensor, ColorSensor *colorSensor, SonarSensor *sonarSensor, Motor *leftWheel, Motor *rightWheel, Motor *armMotor, GyroSensor *gyroSensor, Clock *clock)
{
    this->touchSensor = touchSensor;
    this->colorSensor = colorSensor;
    this->sonarSensor = sonarSensor;
    this->gyroSensor = gyroSensor;
    this->leftWheel = leftWheel;
    this->rightWheel = rightWheel;
    this->armMotor = armMotor;
    this->clock = clock;
}

RobotAPI::~RobotAPI()
{
    delete touchSensor;
    delete colorSensor;
    delete sonarSensor;
    delete gyroSensor;
    delete leftWheel;
    delete rightWheel;
    delete clock;
}

TouchSensor *RobotAPI::getTouchSensor()
{
    return touchSensor;
}

ColorSensor *RobotAPI::getColorSensor()
{
    return colorSensor;
}

SonarSensor *RobotAPI::getSonarSensor()
{
    return sonarSensor;
}

GyroSensor *RobotAPI::getGyroSensor()
{
    return gyroSensor;
}

Motor *RobotAPI::getLeftWheel()
{
    return leftWheel;
}

Motor *RobotAPI::getRightWheel()
{
    return rightWheel;
}

Motor *RobotAPI::getArmMotor()
{
    return armMotor;
}

Clock *RobotAPI::getClock()
{
    return clock;
}