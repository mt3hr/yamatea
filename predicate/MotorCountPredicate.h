#include "Predicate.h"
#include "Motor.h"

using namespace ev3api;

// MotorCountPredicate
// モータ回転数がある値を超えたらtrueを返すPredicate
class MotorCountPredicate : public Predicate
{
private:
    Motor *motor;
    int count;

public:
    MotorCountPredicate(Motor *motor, int count);
    bool test() override;
};