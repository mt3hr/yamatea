#include "SwingDistanceCompensationHandler.h"
#include "math.h"

void SwingDistanceCompensationHandler::handle()
{
    int diffBrightness = abs(targetBrightness - colorSensor->getBrightness());

    if (diffBrightness > 10)
    {
        if (leftWheel->getPWM() > rightWheel->getPWM()) // getPWMメソッドってprotectedなんだって。どうしよ
        {
            leftWheel->setCount(leftWheel->getCount() - 10); // 補正値適当すぎ
        }
        else
        {
            rightWheel->setCount(rightWheel->getCount() - 10); // 補正値適当すぎ
        }
    }
    return;
}

void SwingDistanceCompensationHandler::setTargetBrightness(int tb)
{
    targetBrightness = tb;
}