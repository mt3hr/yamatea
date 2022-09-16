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
#include "PIDTargetColorBrightnessCalibrator.h"

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
    // commandsからキャリブレータを抽出する
    vector<PIDTargetColorBrightnessCalibrator *> calibrators;
    for (int i = 0; i < (int)commands.size(); i++)
    {
        PIDTargetColorBrightnessCalibrator *calibrator = dynamic_cast<PIDTargetColorBrightnessCalibrator *>(commands[i]);
        if (calibrator != NULL)
        {
            calibrators.push_back(calibrator);
        }
    }
    writeDebug("CommandExecutor");
    writeEndLineDebug();
    writeDebug("extructed calibrator");
    flushDebug(DEBUG, robotAPI);

    vector<vector<int> *> *targetBrightnessTracerIndexs = new vector<vector<int> *>();
    vector<vector<int> *> *targetColorTracerIndexs = new vector<vector<int> *>();
    for (int i = 0; i < ((int)commands.size()); i++)
    {
        // commands[i]がcalibrators[i]の対象であるかを判断する
        for (int j = 0; j < ((int)calibrators.size()); j++)
        {
            targetBrightnessTracerIndexs->push_back(new vector<int>());
            targetColorTracerIndexs->push_back(new vector<int>());
            vector<PIDTracer *> targetPIDTracer = *calibrators[j]->getPIDTracers();
            vector<ColorPIDTracer *> targetColorPIDTracer = *calibrators[j]->getColorPIDTracers();
            for (int k = 0; k < ((int)targetPIDTracer.size()); k++)
            {
                if (&(*targetPIDTracer[k]) == &(*commands[i]))
                {
                    (*targetBrightnessTracerIndexs)[j]->push_back(i);
                    writeDebug("pidTracer index: ");
                    writeDebug(i);
                    flushDebug(DEBUG, robotAPI);
                }
            }
            for (int k = 0; k < ((int)targetColorPIDTracer.size()); k++)
            {
                if (&(*targetColorPIDTracer[k]) == &(*commands[i]))
                {
                    (*targetColorTracerIndexs)[j]->push_back(i);
                    writeDebug("colorPIDTracer index: ");
                    writeDebug(i);
                    flushDebug(DEBUG, robotAPI);
                }
            }
        }
    }

    writeDebug("targetBrightnessTracerIndexs size: ");
    writeDebug((int)(*(*targetBrightnessTracerIndexs)[0]).size());
    flushDebug(DEBUG, robotAPI);

    writeDebug("extructed target pid tracer");
    flushDebug(DEBUG, robotAPI);

    for (int i = 0; i < (int)commands.size(); i++)
    {
        commands[i] = commands[i]->generateReverseCommand();
        predicates[i] = predicates[i]->generateReversePredicate();
    }
    writeDebug("reversed command");
    flushDebug(DEBUG, robotAPI);

    vector<PIDTargetColorBrightnessCalibrator *> newCalibrators;
    for (int i = 0; i < (int)commands.size(); i++)
    {
        PIDTargetColorBrightnessCalibrator *calibrator = dynamic_cast<PIDTargetColorBrightnessCalibrator *>(commands[i]);
        if (calibrator != NULL)
        {
            newCalibrators.push_back(calibrator);
        }
    }

    writeDebug("reversedCalibrator.size():");
    writeDebug((int)newCalibrators.size());
    writeEndLineDebug();
    writeDebug("extructed reversed calibrator");
    flushDebug(DEBUG, robotAPI);

    // PIDCalibratorの対象を新キャリブレータに適用する
    for (int j = 0; j < ((int)newCalibrators.size()); j++)
    {
        writeDebug("targetBrightnessTracerIndexsSize: ");
        writeDebug(j);
        writeDebug(": ");
        writeDebug((int)(*targetBrightnessTracerIndexs)[j]->size());
        flushDebug(DEBUG, robotAPI);
        for (int k = 0; k < ((int)(*targetBrightnessTracerIndexs)[j]->size()); k++)
        {
            PIDTracer *pidTracer = dynamic_cast<PIDTracer *>(commands[(*(*targetBrightnessTracerIndexs)[j])[k]]);
            writeDebug("pidTracer is null: ");
            writeDebug(pidTracer == NULL);
            flushDebug(DEBUG, robotAPI);
            newCalibrators[j]->addPIDTracer(pidTracer);
            writeDebug("set pid tracer to calibrator");
            flushDebug(DEBUG, robotAPI);
        }
    }
    for (int j = 0; j < ((int)newCalibrators.size()); j++)
    {
        writeDebug("targetColorTracerIndexsSize: ");
        writeDebug(j);
        writeDebug(": ");
        writeDebug((int)(*targetColorTracerIndexs)[j]->size());
        flushDebug(DEBUG, robotAPI);
        for (int k = 0; k < ((int)(*targetColorTracerIndexs)[j]->size()); k++)
        {
            ColorPIDTracer *colorPIDTracer = dynamic_cast<ColorPIDTracer *>(commands[(*(*targetColorTracerIndexs)[j])[k]]);
            writeDebug("colorPIDTracer is null: ");
            writeDebug(colorPIDTracer == NULL);
            flushDebug(DEBUG, robotAPI);
            newCalibrators[j]->addColorPIDTracer(colorPIDTracer);
            writeDebug("set color pid tracer to calibrator");
            flushDebug(DEBUG, robotAPI);
        }
    }
    writeDebug("set pid tracers to calibrator");
    flushDebug(DEBUG, robotAPI);

    // predicateがFinishConfirmableだったら対象を反転後のコマンドに変更する
    for (int i = 0; i < ((int)commands.size()); i++)
    {
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
    writeDebug("resoluted finish confirmable commands");
    flushDebug(DEBUG, robotAPI);
}

bool CommandExecutor::isFinished()
{
    return !(((int)commands.size()) > ((int)currentIndexForCommand));
}