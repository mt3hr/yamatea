#ifndef CommandExecutor_H
#define CommandExecutor_H

#include "Predicate.h"
#include "Command.h"
#include <vector>
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
    vector<bool> preparated;
    RobotAPI *robotAPI;

public:
    CommandExecutor(RobotAPI *robotAPI);
    virtual ~CommandExecutor();
    virtual void addCommand(Command *command, Predicate *exitCondition);
    virtual void run();
    virtual void emergencyStop();
    virtual void reverseCommandAndPredicate();
};

#endif
