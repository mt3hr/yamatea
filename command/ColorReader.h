#ifndef ColorReader_H
#define ColorReader_H

#include "Command.h"
#include "RobotAPI.h"
#include "Sensor.h"

// 非推奨。ColorIDを直に読み取ると精度が悪い。
// 代わりにColorReaderUseRawを使ってください。
// ColorReader
// カラーセンサから色情報を取得するコマンド
//
// 実方
class ColorReader : public Command
{
private:
    colorid_t *color;

public:
    ColorReader();
    virtual ~ColorReader();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ColorReader *generateReverseCommand() override;
    virtual colorid_t *getColorPtr();
};

#endif
