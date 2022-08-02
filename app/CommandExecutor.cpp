#include "Motor.h"
#include "CommandExecutor.h"
#include "Command.h"
#include "Handler.h"

using namespace std;

CommandExecutor::CommandExecutor(Motor *lw, Motor *rw)
{
    leftWheel = lw;
    rightWheel = rw;
}

void CommandExecutor::addCommand(Command *command, Predicate *exitCondition)
{
    commands.push_back(command);
    predicates.push_back(exitCondition);
}

void CommandExecutor::run()
{
    // 終了条件が満たされたらindexを変更して次のコマンドに移動する
    if (((int)sizeof(predicates)) - 1 > ((int)(currentIndexForCommand)) &&
        predicates[currentIndexForCommand]->test())
    {
        currentIndexForCommand++;
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
    leftWheel->stop();
    rightWheel->stop();
    currentIndexForCommand = commands.size();
}