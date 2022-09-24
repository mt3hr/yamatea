#ifndef CorrectedMotor_H
#define CorrectedMotor_H

#include "Motor.h"

using namespace ev3api;

class CorrectedMotor
{
private:
    Motor *motor;
    float pwmCorrectionValue;

public:
    CorrectedMotor(Motor *motor, float pwmCorrectionValue);
    virtual ~CorrectedMotor();
    void reset(void);
    int32_t getCount(void) const;
    void setCount(int32_t count);
    void setPWM(int pwm);
    void setBrake(bool brake);
    void stop();
};

#endif