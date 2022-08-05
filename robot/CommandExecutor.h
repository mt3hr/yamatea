#ifndef CommandExecutor_H
#define CommandExecutor_H

#include "WheelController.h"
#include "Predicate.h"
#include "Command.h"
#include "Handler.h"
#include <vector>

using namespace std;
using namespace ev3api;

// CommandExecutor
// Commandの実行者。
// ロボット操作命令を逐次実行していくもの。
// メインタスクから呼び出される。
//
// タッチセンサーが押されるまでpow20で直進するサンプルを下に示す。
//
// CommandExecutor *commandExecutor = new CommandExecutor();
// int leftPow = 20;
// int rightPow = 20;
// ScenarioTracer *commandScenarioTracer = new ScenarioTracer(leftPow, rightPow, &leftWheel, &rightWheel);
// Predicate *startButtonPredicate = new StartButtonPredicate(&touchSensor);
// commandExecutor->addCommand(commandScenarioTracer, startButtonPredicate);
// commandExecutor->run();
//
// 実方
class CommandExecutor
{
private:
    int currentIndexForCommand;
    vector<Command *> commands;
    vector<Predicate *> predicates;
    vector<Handler *> exitHandlers;
    WheelController *wheelController;

public:
    CommandExecutor(WheelController *wheelController);
    ~CommandExecutor();
    void addCommand(Command *command, Predicate *exitCondition, Handler *exitHandler);
    void run();
    void emergencyStop();
};

#endif