#include "Preparizable.h"
#include "ExecutePreparationWhenExitBeforeCommandHandler.h"
#include "RobotAPI.h"

ExecutePreparationWhenExitBeforeCommandHandler::ExecutePreparationWhenExitBeforeCommandHandler(Preparizable *p)
{
    preparizable = p;
}

void ExecutePreparationWhenExitBeforeCommandHandler::handle(RobotAPI *robotAPI)
{
    preparizable->preparation();
}