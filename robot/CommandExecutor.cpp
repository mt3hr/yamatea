#include "Motor.h"
#include "CommandExecutor.h"
#include "Command.h"
#include "Handler.h"

using namespace std;

CommandExecutor::CommandExecutor(WheelController *wc)
{
    wheelController = wc;
}

CommandExecutor::~CommandExecutor()
{
    for (int i = 0; i < ((int)sizeof(commands)); i++)
    {
        delete (commands[i]);
    }
    for (int i = 0; i < ((int)sizeof(predicates)); i++)
    {
        delete (predicates[i]);
    }
    for (int i = 0; i < ((int)sizeof(exitHandlers)); i++)
    {
        delete (exitHandlers[i]);
    }
}

void CommandExecutor::addCommand(Command *command, Predicate *exitCondition, Handler *exitHandler)
{
    commands.push_back(command);
    predicates.push_back(exitCondition);
    exitHandlers.push_back(exitHandler);
}

void CommandExecutor::run()
{
    // 終了条件が満たされたらindexを変更して次のコマンドに移動する
    if (((int)sizeof(predicates)) - 1 > ((int)(currentIndexForCommand)) && predicates[currentIndexForCommand]->test())
    {
        exitHandlers[currentIndexForCommand]->handle();
        currentIndexForCommand++;
        return;
    }

    // 現在の要素が有ればやる。なければ何もせずに返す
    if (((int)sizeof(commands)) - 1 > ((int)currentIndexForCommand))
    {
        commands[currentIndexForCommand]->run();
    }

    return;
}

void CommandExecutor::emergencyStop()
{
    wheelController->getLeftWheel()->stop();
    wheelController->getRightWheel()->stop();
    currentIndexForCommand = commands.size();
}
