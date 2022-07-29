#include "CommandExecutor.h"
#include "Command.h"
#include "Handler.h"

using namespace std;

CommandExecutor::CommandExecutor()
{
}

void CommandExecutor::addCommand(Command *command, Predicate *exitCondition)
{
    commandVector.push_back(command);
    predicateVector.push_back(exitCondition);
}

void CommandExecutor::addCyclicHandler(Handler *handler)
{
    handlerVector.push_back(handler);
}

void CommandExecutor::run()
{
    // 終了条件が満たされたらindexを変更して次のコマンドに移動する
    if (((int)sizeof(predicateVector)) - 1 > ((int)(currentIndexForCommand)) &&
        predicateVector[currentIndexForCommand]->test())
    {
        currentIndexForCommand++;
    }

    // 現在の要素が有ればやる。なければ何もせずに返す
    if (((int)sizeof(commandVector)) - 1 > ((int)currentIndexForCommand))
    {
        commandVector[currentIndexForCommand]->run();
    }

    //TODO ハンドラを走らせる
    /*
    for (int i = 0; i < ((int)sizeof(handlerVector)); i++)
    {
        handlerVector[i]->handle();
    }
    */
    return;
}