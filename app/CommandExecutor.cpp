#include "CommandExecutor.h"
#include "Command.h"

using namespace std;

CommandExecutor::CommandExecutor()
{
}

void CommandExecutor::addCommand(Command *command, Predicate *exitCondition)
{
    commandVector.push_back(command);
    predicateVector.push_back(exitCondition);
}

void CommandExecutor::run()
{
    // 終了条件が満たされたらindexを変更して次のコマンドに移動する
    if (((int)sizeof(predicateVector)) - 1 > ((int)(currentIndex)) &&
        predicateVector[currentIndex]->test())
    {
        currentIndex++;
    }

    // 現在の要素が有ればやる。なければ何もせずに返す
    if (((int)sizeof(commandVector)) - 1 > ((int)currentIndex))
    {
        commandVector[currentIndex]->run();
    }
    return;
}