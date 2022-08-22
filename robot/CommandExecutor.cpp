#include "CommandExecutor.h"
#include "ev3api.h"
#include "Motor.h"
#include "Command.h"
#include "Stopper.h"
#include "PrintMessage.h"
#include "RobotAPI.h"
#include "Setting.h"
#include "DebugUtil.h"

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
}

void CommandExecutor::addCommand(Command *command, Predicate *exitCondition)
{
    commands.push_back(command);
    predicates.push_back(exitCondition);
    preparated.push_back(false);
}

void CommandExecutor::run()
{
    // 完了していればなにもしないで返す
    if (finished)
    {
        return;
    }

    // Commandがはじめて実行される時にPrediate.preparation()メソッドを実行する
    if (!preparated[currentIndexForCommand])
    {
        preparated[currentIndexForCommand] = true;
        commands[currentIndexForCommand]->preparation(robotAPI);
        predicates[currentIndexForCommand]->preparation(robotAPI);
    }

    // 終了条件が満たされたらindexを変更して次のコマンドに移動する
    if (predicates[currentIndexForCommand]->test(robotAPI))
    {
        currentIndexForCommand++;
        beepDebug();
    }

    if (((int)commands.size()) > ((int)currentIndexForCommand))
    {
        // コマンドを実行する
        commands[currentIndexForCommand]->run(robotAPI);
    }
    else
    {
        // 現在の要素がなければ停止してタスクを終了する。
        finished = true;
        stp_cyc(RUNNER_CYC);
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
        return;
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
