#ifndef Note_H
#define Note_H
#include "ev3api.h"

#include "Command.h"
#include "FinishConfirmable.h"
#include "RobotAPI.h"

using namespace ev3api;

// Note
// 1つの音符を表現するクラス。
// runすると対応したBeep音を鳴らす。
//
// 実方
class Note : public Command, public FinishConfirmable
{
private:
    uint16_t frequency;
    int32_t duration;
    uint64_t duration64t;
    uint8_t volume;

    uint64_t targetTime = 0;
    bool beeped = false;
    bool finished = false;

public:
    Note(uint16_t frequency, int32_t duration, uint8_t volume);
    virtual ~Note();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual bool isFinished() override;
    virtual Command *generateReverseCommand() override;
    virtual uint16_t getFrequency();
    virtual int32_t getDuration();
    virtual uint8_t getVolume();
};

#endif