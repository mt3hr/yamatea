#ifndef Hedgehog_H
#define Hedgehog_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Stopper.h"
#include "RobotAPI.h"

// Hedgehog
// ソナーセンサで取得できる値が指定距離になるまで直進、後退を繰り返すコマンド。
//
// 実方
class Hedgehog : public Command, public FinishConfirmable
{
private:
    int distance;
    int targetDistance;
    int pwm;
    Stopper *stopper;

public:
    Hedgehog(int targetDistance, int pwm);
    virtual ~Hedgehog();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Hedgehog *generateReverseCommand() override;
    virtual bool isFinished() override;
};

#endif