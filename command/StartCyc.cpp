#include "StartCyc.h"
#include "Command.h"
#include "RobotAPI.h"
#include "string"
#include "ev3api.h"

using namespace ev3api;

StartCyc::StartCyc(ID taskName)
{
    this->taskName = taskName;
};

StartCyc::~StartCyc(){

};

void StartCyc::run(RobotAPI *robotAPI)
{
    sta_cyc(taskName);
    return;
}

void StartCyc::preparation(RobotAPI *robotAPI)
{
    return;
}

StartCyc *StartCyc::generateReverseCommand()
{
    return new StartCyc(taskName);
}