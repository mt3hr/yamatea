#ifndef RobotAPI_H
#define RobotAPI_H

#include "TouchSensor.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "Motor.h"
#include "Clock.h"

using namespace ev3api;

class RobotAPI
{
private:
    TouchSensor *touchSensor;
    ColorSensor *colorSensor;
    SonarSensor *sonarSensor;
    Motor *leftWheel;
    Motor *rightWheel;
    Motor *armMotor;
    Clock *clock;

public:
    RobotAPI(TouchSensor *TouchSensor, ColorSensor *colorSensor, SonarSensor *sonarSensor, Motor *leftWheel, Motor *rightWheel, Motor *armMotor, Clock *clock);
    ~RobotAPI();
    TouchSensor *getTouchSensor();
    ColorSensor *getColorSensor();
    SonarSensor *getSonarSensor();
    Motor *getLeftWheel();
    Motor *getRightWheel();
    Motor *getArmMotor();
    Clock *getClock();
};

#endif