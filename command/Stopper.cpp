#include "Stopper.h"
#include "PrintMessage.h"
#include "Setting.h"

using namespace ev3api;

Stopper::Stopper(WheelController *wc)
{
    wheelController = wc;
}

void Stopper::run()
{
    wheelController->getLeftWheel()->stop();
    wheelController->getRightWheel()->stop();

    vector<string> messageLines;
    messageLines.push_back("stoped.");
    PrintMessage printMessagee(messageLines, false);
    printMessagee.run();
}

Stopper *Stopper::generateReverseCommand()
{
    return new Stopper(wheelController);
}