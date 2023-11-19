// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#include "Setting.h"
#ifdef GoalSanekataPIDMode1
#ifndef GoalSanekataPIDMode1_
#define GoalSanekataPIDMode1_
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI);
#endif
#endif