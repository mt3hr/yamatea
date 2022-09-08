#include "Stopper.h"
#include "RobotAPI.h"
#include "Setting.h"
#include "DebugUtil.h"

using namespace ev3api;

Stopper::Stopper()
{
}

Stopper::~Stopper()
{
}

void Stopper::run(RobotAPI *robotAPI)
{
    robotAPI->getLeftWheel()->stop();
    robotAPI->getRightWheel()->stop();
    robotAPI->getArmMotor()->stop();
    robotAPI->getTailMotor()->stop();

    ev3_speaker_set_volume(0);
    ev3_speaker_play_tone(beepNoteWhenCommandSwitching->getFrequency(), beepNoteWhenCommandSwitching->getDuration());

    writeDebug("Stopper");
    writeEndLineDebug();
    writeDebug("stopped.");
    flushDebug(INFO, robotAPI);
}

void Stopper::preparation(RobotAPI *robotAPI)
{
    return;
}

Stopper *Stopper::generateReverseCommand()
{
    return new Stopper();
}