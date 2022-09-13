#ifndef CommandExecutor_H
#define CommandExecutor_H

#include "Predicate.h"
#include "Command.h"
#include <vector>
#include "string"
#include "RobotAPI.h"

using namespace std;
using namespace ev3api;

// CommandExecutor
// Commandの実行者。
// ロボット操作命令を逐次実行していくもの。
// メインタスクから呼び出される。
//
// 実方
class CommandExecutor
{
private:
    int currentIndexForCommand;
    bool finished = false;
    vector<Command *> commands;
    vector<Predicate *> predicates;
    vector<string> commandNames;
    vector<bool> preparated;
    RobotAPI *robotAPI;
    bool runner;
    uint64_t loopCount = 0;
    uint64_t time = 0;

public:
    CommandExecutor(RobotAPI *robotAPI, bool runner);
    virtual ~CommandExecutor();
    virtual void addCommand(Command *command, Predicate *exitCondition, string commandName);
    virtual void run();
    virtual void nextCommand();
    virtual void emergencyStop();
    virtual void reverseCommandAndPredicate();
    virtual bool isFinished();
};

#endif
