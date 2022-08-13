#include "RobotAPI.h"

RobotAPI::RobotAPI(TouchSensor *touchSensor, ColorSensor *colorSensor, SonarSensor *sonarSensor, Motor *leftWheel, Motor *rightWheel, Motor *armMotor, Clock *clock)
{
    this->touchSensor = touchSensor;
    this->colorSensor = colorSensor;
    this->sonarSensor = sonarSensor;
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