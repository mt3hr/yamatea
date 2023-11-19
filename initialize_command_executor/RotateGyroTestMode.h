// NOTE ジャイロ、 実機とシミュレータで左右判定が逆になる？
#include "Setting.h"
#ifdef RotateGyroTestMode
#ifndef RotateGyroTestMode_
#define RotateGyroTestMode_
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI);
#endif
#endif