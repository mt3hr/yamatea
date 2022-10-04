#include "CorrectedMotor.h"
#include "Motor.h"
#include "math.h"

using namespace ev3api;

CorrectedMotor::CorrectedMotor(Motor *motor, float pwmCorrectionValue)
{
    this->motor = motor;
    this->pwmCorrectionValue = pwmCorrectionValue;
}

CorrectedMotor::~CorrectedMotor()
{
}

void CorrectedMotor::reset(void)
{
    motor->reset();
}

int32_t CorrectedMotor::getCount(void) const
{
    return motor->getCount();
}

void CorrectedMotor::setCount(int32_t count)
{
    motor->setCount(count);
}

void CorrectedMotor::setPWM(float pwm)
{
    motor->setPWM((int)(round(pwm * pwmCorrectionValue)));
}

void CorrectedMotor::setBrake(bool brake)
{
    motor->setBrake(brake);
}

void CorrectedMotor::stop()
{
    motor->stop();
}