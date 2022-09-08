#include "RobotAPI.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "DebugUtil.h"

using namespace ev3api;

RobotAPI::RobotAPI(TouchSensor *touchSensor, ColorSensor *colorSensor, SonarSensor *sonarSensor, Motor *leftWheel, Motor *rightWheel, Motor *armMotor, Motor *tailMotor, GyroSensor *gyroSensor, Clock *clock)
{
    this->touchSensor = touchSensor;
    this->colorSensor = colorSensor;
    this->sonarSensor = sonarSensor;
    this->gyroSensor = gyroSensor;
    this->leftWheel = leftWheel;
    this->rightWheel = rightWheel;
    this->armMotor = armMotor;
    this->clock = clock;
    this->tailMotor = tailMotor;
    this->measAngle = new RobotAPI::MeasAngleUseWheel(this);
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

Motor *RobotAPI::getTailMotor()
{
    return tailMotor;
}

RobotAPI::MeasAngleUseWheel *RobotAPI::getMeasAngle()
{
    return measAngle;
}

void RobotAPI::reset()
{
    gyroSensor->reset();
    leftWheel->reset();
    rightWheel->reset();
    armMotor->reset();
    clock->reset();
    tailMotor->reset();
}

RobotAPI::MeasAngleUseWheel::MeasAngleUseWheel(RobotAPI *robotAPI)
{
    this->robotAPI = robotAPI;
};

RobotAPI::MeasAngleUseWheel::~MeasAngleUseWheel(){};

float RobotAPI::MeasAngleUseWheel::getAngle()
{
    int leftCount = robotAPI->getLeftWheel()->getCount();
    int rightCount = robotAPI->getRightWheel()->getCount();

    int diffCount = (leftCount - rightCount);
    int modDiffCount = diffCount;
    angle = float(modDiffCount) / (float(angleFor360TurnMeasAngle) / float(360));
    writeDebug("meas angle use wheel");
    writeEndLineDebug();
    writeDebug("diffCount: ");
    writeDebug(diffCount);
    writeEndLineDebug();
    writeDebug("angle: ");
    writeDebug(angle);
    flushDebug(TRACE, robotAPI);
    return angle + angleOffset;
}

void RobotAPI::MeasAngleUseWheel::reset()
{
    angleOffset = -getAngle();
}