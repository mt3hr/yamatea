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
#include "FinishedCommandPredicate.h"
#include "typeinfo"
#include "FinishConfirmable.h"

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
    if (isFinished())
    {
        return;
    }

    loopCount++;

#ifdef EnableRunnerTaskTimeCheck
    time = robotAPI->getClock()->now();
    writeDebug("runner task");
    writeDebug(loopCount);
    writeDebug("start: ");
    writeDebug(time);
    writeDebug("usec");
    flushDebug(NONE, robotAPI);
#endif

    // Commandがはじめて実行される時にPrediate.preparation()メソッドを実行する
    if (!preparated[currentIndexForCommand])
    {
        preparated[currentIndexForCommand] = true;
        commands[currentIndexForCommand]->preparation(robotAPI);
        predicates[currentIndexForCommand]->preparation(robotAPI);

#ifdef EnablePrintCommandName
        vector<string> messageLines;
        messageLines.push_back("STARTED " + commandNames[currentIndexForCommand]);
        PrintMessage *printMessage = new PrintMessage(messageLines, true);
        printMessage->run(robotAPI);
        delete printMessage;
#endif
    }

    // 終了条件が満たされたらindexを変更して次のコマンドに移動する
    if (predicates[currentIndexForCommand]->test(robotAPI))
    {
        nextCommand();
    }

    if (!isFinished())
    {
        // コマンドを実行する
        commands[currentIndexForCommand]->run(robotAPI);
        if (runner)
        {
#ifdef EnablePrintGyroValue
#ifdef SimulatorMode
            int gyroAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
            int gyroAngle = robotAPI->getGyroSensor()->getAngle();
#endif
            writeDebug("gyro angle: ");
            writeDebug(gyroAngle);
            flushDebug(TRACE, robotAPI);
#endif
#ifdef EnablePrintAngleUseWheel
            writeDebug("meased angle: ");
            writeDebug(robotAPI->getMeasAngle()->getAngle());
            flushDebug(TRACE, robotAPI);
#endif
#ifdef EnablePrintMotorCount
            writeDebug("left wheel count: ");
            writeDebug(robotAPI->getLeftWheel()->getCount());
            writeEndLineDebug();
            writeDebug("right wheel count: ");
            writeDebug(robotAPI->getRightWheel()->getCount());
            flushDebug(TRACE, robotAPI);
#endif
        }
    }

#ifdef EnableRunnerTaskTimeCheck
    time = robotAPI->getClock()->now();
    writeDebug("runner task");
    writeDebug(loopCount);
    writeDebug("finish: ");
    writeDebug(time);
    writeDebug("usec");
    flushDebug(NONE, robotAPI);
#endif
    return;
}

void CommandExecutor::nextCommand()
{
    currentIndexForCommand++;

    if (isFinished())
    {
        // 現在の要素がなければ停止してタスクを終了する。
        if (runner)
        {
            Stopper *stopper = new Stopper();
            stopper->run(robotAPI);
            delete stopper;
        }
        return;
    }

    if (enableBeepWhenCommandSwitching)
    {
        beepDebug();
    }
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
}

void CommandExecutor::reverseCommandAndPredicate()
{
    FinishedCommandPredicate *fcp = new FinishedCommandPredicate(new FinishConfirmable());
    string finishCommandPredicateTypeName = typeid(fcp).name();

    for (int i = 0; i < ((int)commands.size()); i++)
    {
        commands[i] = commands[i]->generateReverseCommand();
        predicates[i] = predicates[i]->generateReversePredicate();

        FinishConfirmable *newFinishConfirmable = dynamic_cast<FinishConfirmable *>(commands[i]);
        if (newFinishConfirmable != NULL)
        {
            Command *newCommand = dynamic_cast<Command *>(commands[i]);
            FinishedCommandPredicate *newPredicate = dynamic_cast<FinishedCommandPredicate *>(predicates[i]);
            newPredicate->setTarget(newFinishConfirmable);

            commands[i] = newCommand;
            predicates[i] = newPredicate;
        }
    }
}

bool CommandExecutor::isFinished()
{
    return !(((int)commands.size()) > ((int)currentIndexForCommand));
}