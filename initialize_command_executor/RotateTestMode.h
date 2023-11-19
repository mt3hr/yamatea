// RotateTestModeの場合のcommandExecutor初期化処理
#include "Setting.h"
#ifdef RotateTestMode
#ifndef RotateTestMode_
#define RotateTestMode_
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI);
#endif
#endif