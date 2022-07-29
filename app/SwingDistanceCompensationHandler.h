#include "ColorSensor.h"
#include "Motor.h"
#include "Handler.h"

using namespace ev3api;

// SwingDistanceCompensationHandler 
// 首振りしたときのモーター回転数を補正するハンドラ。
// CommandExecutorに追加して使う。
// 
// 実方
class SwingDistanceCompensationHandler : public Handler
{
private:
    ColorSensor *colorSensor;
    Motor *leftWheel;
    Motor *rightWheel;
    int targetBrightness;

public:
    SwingDistanceCompensationHandler(ColorSensor *colorSensor, Motor *leftWheel, Motor *rightWheel);
    void handle() override;
    void setTargetBrightness(int targetBrightness);
};

SwingDistanceCompensationHandler::SwingDistanceCompensationHandler(ColorSensor *cs, Motor *lw, Motor *rw)
{
    colorSensor = cs;
    leftWheel = lw;
    rightWheel = rw;
}
