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
    commandVector.push_back(command);
    predicateVector.push_back(exitCondition);
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

    return;
}

void CommandExecutor::emergencyStop()
{
    leftWheel->stop();
    rightWheel->stop();
    currentIndexForCommand = commandVector.size();
}