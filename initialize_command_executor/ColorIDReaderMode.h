// ColorIDReaderModeの場合のcommandExecutor初期化処理
#include "Setting.h"
#ifdef ColorIDReaderMode
#ifndef ColorIDReaderMode_
#define ColorIDReaderMode_
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI);
#endif
#endif