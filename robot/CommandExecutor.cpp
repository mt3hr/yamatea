#include "CommandExecutor.h"
#include "ev3api.h"
#include "Motor.h"
#include "Command.h"
#include "Handler.h"
#include "Stopper.h"
#include "PrintMessage.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

CommandExecutor::CommandExecutor(RobotAPI *robotAPI)
{
    this->robotAPI = robotAPI;
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
    if (((int)predicates.size()) > ((int)(currentIndexForCommand)) && predicates[currentIndexForCommand]->test(robotAPI))
    {
        exitHandlers[currentIndexForCommand]->handle(robotAPI);
        currentIndexForCommand++;
        return;
    }

    // 現在の要素が有ればやる。なければタスクを終了する。
    if (((int)commands.size()) > ((int)currentIndexForCommand))
    {
        commands[currentIndexForCommand]->run(robotAPI);
    }
    else
    {
        stp_cyc(RUNNER_CYC);
    }

    return;
}

void CommandExecutor::emergencyStop()
{
    currentIndexForCommand = commands.size() + 1;
    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;

    vector<string> messageLines;
    messageLines.push_back("emergency stopped");
    PrintMessage printStopMessage(messageLines, true);
    printStopMessage.run(robotAPI);
    stp_cyc(RUNNER_CYC);
}
