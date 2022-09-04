#include "Note.h"
#include "RobotAPI.h"
#include "DebugUtil.h"
#include "ev3api.h"

using namespace ev3api;

Note::Note(uint16_t frequency, int32_t duration, uint8_t volume)
{
    this->frequency = frequency;
    this->duration = duration;
    this->volume = volume;
};

Note::~Note(){};

void Note::run(RobotAPI *robotAPI)
{
    if (!beeped)
    {
        targetTime = robotAPI->getClock()->now() + uint64_t(duration * 1000);
        beeped = true;
        ev3_speaker_set_volume(getVolume());
        ev3_speaker_play_tone(getFrequency(), getDuration());
    }

    finished = targetTime <= robotAPI->getClock()->now();
    return;
}

void Note::preparation(RobotAPI *robotAPI)
{
    targetTime = robotAPI->getClock()->now() + uint64_t(duration * 1000);
}

bool Note::isFinished()
{
    return finished;
}

Command *Note::generateReverseCommand()
{
    return new Note(frequency, duration, volume);
}

uint16_t Note::getFrequency()
{
    return frequency;
}

int32_t Note::getDuration()
{
    return duration;
}

uint8_t Note::getVolume()
{
    return volume;
}