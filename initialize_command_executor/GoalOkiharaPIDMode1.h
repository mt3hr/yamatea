// LeftCourceMode, RightCourceModeの場合のcommandExecutor初期化処理
#include "Setting.h"
#ifdef GoalOkiharaPIDMode1
#ifndef GoalOkiharaPIDMode1_
#define GoalOkiharaPIDMode1_
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI);
#endif
#endif