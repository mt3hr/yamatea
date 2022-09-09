#include "CommandExecutor.h"
#include "ev3api.h"
#include "Motor.h"
#include "Command.h"
#include "Stopper.h"
#include "PrintMessage.h"
#include "RobotAPI.h"
#include "Setting.h"
#include "DebugUtil.h"
#include "string"
#include "sstream"
#include "vector"

using namespace ev3api;
using namespace std;

CommandExecutor::CommandExecutor(RobotAPI *robotAPI, bool runner)
{
    this->runner = runner;
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

void CommandExecutor::addCommand(Command *command, Predicate *exitCondition, string commandName)
{
    commands.push_back(command);
    predicates.push_back(exitCondition);
    commandNames.push_back(commandName);
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

        vector<string> messageLines;
        messageLines.push_back("STARTED " + commandNames[currentIndexForCommand]);
        PrintMessage *printMessage = new PrintMessage(messageLines, true);
        printMessage->run(robotAPI);
        delete printMessage;
    }

    // 終了条件が満たされたらindexを変更して次のコマンドに移動する
    if (predicates[currentIndexForCommand]->test(robotAPI))
    {
        nextCommand();
        if (enableBeepWhenCommandSwitching)
        {
            beepDebug();
        }
    }

    if (((int)commands.size()) > ((int)currentIndexForCommand))
    {
        // コマンドを実行する
        commands[currentIndexForCommand]->run(robotAPI);
#ifdef EnablePrintGyroValue
        writeDebug("gyro angle: ");
        writeDebug(robotAPI->getGyroSensor()->getAngle());
        flushDebug(INFO, robotAPI);
#endif
#ifdef EnablePrintAngleUseWheel
        writeDebug("angle: ");
        writeDebug(robotAPI->getMeasAngle()->getAngle());
        flushDebug(INFO, robotAPI);
#endif
#ifdef EnablePrintMotorCount
        writeDebug("left wheel count: ");
        writeDebug(robotAPI->getLeftWheel()->getCount());
        writeEndLineDebug();
        writeDebug("right wheel count: ");
        writeDebug(robotAPI->getRightWheel()->getCount());
        flushDebug(INFO, robotAPI);
#endif
    }
    else
    {
        // 現在の要素がなければ停止してタスクを終了する。
        finished = true;
        if (runner)
        {
            stp_cyc(RUNNER_CYC);
            Stopper *stopper = new Stopper();
            stopper->run(robotAPI);
            delete stopper;
        }
        return;
    }

    return;
}

void CommandExecutor::nextCommand()
{
    currentIndexForCommand++;
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

void CommandExecutor::reverseCommandAndPredicate()
{
    for (int i = 0; i < ((int)sizeof(commands)); i++)
    {
        Command *reversed = commands[i]->generateReverseCommand();
        delete commands[i];
        commands[i] = reversed;
    }

    for (int i = 0; i < ((int)sizeof(predicates)); i++)
    {
        Predicate *reversed = predicates[i]->generateReversePredicate();
        delete predicates[i];
        predicates[i] = reversed;
    }
}