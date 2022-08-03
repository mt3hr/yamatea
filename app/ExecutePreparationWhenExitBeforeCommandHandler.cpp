#include "Preparizable.h"
#include "ExecutePreparationWhenExitBeforeCommandHandler.h"

ExecutePreparationWhenExitBeforeCommandHandler::ExecutePreparationWhenExitBeforeCommandHandler(Preparizable *p)
{
    preparizable = p;
}

void ExecutePreparationWhenExitBeforeCommandHandler::handle()
{
    preparizable->preparation();
}