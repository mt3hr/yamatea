#ifndef RobotAPI_H
#define RobotAPI_H

#include "RobotAPI.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"

using namespace ev3api;

// RobotAPI
// ロボットのAPIをまとめたクラス
//
// 実方
class RobotAPI
{
public:
    class MeasAngleUseWheel
    {
    private:
        RobotAPI *robotAPI;
        float angle = 0;
        float angleOffset = 0;

    public:
        MeasAngleUseWheel(RobotAPI *robotAPI);
        virtual ~MeasAngleUseWheel();
        virtual float getAngle();
        virtual void reset();
    };

private:
    TouchSensor *touchSensor;
    ColorSensor *colorSensor;
    SonarSensor *sonarSensor;
    GyroSensor *gyroSensor;
    Motor *leftWheel;
    Motor *rightWheel;
    Motor *armMotor;
    Clock *clock;
    Motor *tailMotor;
    RobotAPI::MeasAngleUseWheel *measAngle;

public:
    RobotAPI(TouchSensor *touchSensor, ColorSensor *colorSensor, SonarSensor *sonarSensor, Motor *leftWheel, Motor *rightWheel, Motor *armMotor, Motor *tailMotor, GyroSensor *gyroSensor, Clock *clock);
    virtual ~RobotAPI();
    virtual TouchSensor *getTouchSensor();
    virtual ColorSensor *getColorSensor();
    virtual SonarSensor *getSonarSensor();
    virtual GyroSensor *getGyroSensor();
    virtual Motor *getLeftWheel();
    virtual Motor *getRightWheel();
    virtual Motor *getArmMotor();
    virtual Motor *getTailMotor();
    virtual Clock *getClock();
    virtual RobotAPI::MeasAngleUseWheel * getMeasAngle();
    virtual void reset();
};

#endif
