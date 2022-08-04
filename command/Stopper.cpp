#include "Stopper.h"

using namespace ev3api;

Stopper::Stopper(WheelController *wc)
{
    wheelController = wc;
}

void Stopper::run()
{
    wheelController->getLeftWheel()->stop();
    wheelController->getRightWheel()->stop();
}

Stopper *Stopper::generateReverseCommand()
{
    return new Stopper(wheelController);
}