// DistanceReaderModeの場合のcommandExecutor初期化処理
#include "Setting.h"
#ifdef DistanceReaderMode
#ifndef DistanceReaderMode_
#define DistanceReaderMode_
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI);
#endif
#endif