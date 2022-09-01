#include "Note.h"
#include "RobotAPI.h"

Note::Note(uint16_t frequency, int32_t duration, uint8_t volume)
{
    this->frequency = frequency;
    this->duration = duration;
    this->duration64t = uint64_t(duration) * 1000;
    this->volume = volume;
};

Note::~Note(){};

void Note::run(RobotAPI *robotAPI)
{
    if (!beeped)
    {
        beeped = true;
        ev3_speaker_set_volume(getVolume());
        ev3_speaker_play_tone(getFrequency(), getVolume());
    }

    finished = startTime - robotAPI->getClock()->now() >= duration64t;
    return;
}

void Note::preparation(RobotAPI *robotAPI)
{
    startTime = robotAPI->getClock()->now();
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