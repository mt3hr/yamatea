#include "Stopper.h"
#include "Setting.h"
#include "DebugUtil.h"

using namespace ev3api;

Stopper::Stopper(WheelController *wc)
{
    wheelController = wc;
}

void Stopper::run()
{
    wheelController->getLeftWheel()->stop();
    wheelController->getRightWheel()->stop();

    writeDebug("Stopper");
    writeEndLineDebug();
    writeDebug("stopped.");
    flushDebug();
}

Stopper *Stopper::generateReverseCommand()
{
    return new Stopper(wheelController);
}