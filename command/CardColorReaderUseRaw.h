#ifndef CardColorReaderUseRaw_H
#define CardColorReaderUseRaw_H

#include "Command.h"
#include "RobotAPI.h"
#include "RawColorPredicate.h"

using namespace ev3api;

// CardColorReaderUseRaw
// カラーセンサからガレージカードの色情報を取得するコマンド
// ColorReaderとは違い、キャリブレーションした値を用いる。
//
// 実方
class CardColorReaderUseRaw : public Command
{
private:
    colorid_t *color;
    RawColorPredicate *redCardPredicate = new RedCardPredicate();
    RawColorPredicate *greenCardPredicate = new GreenCardPredicate();
    RawColorPredicate *blueCardPredicate = new BlueCardPredicate();
    RawColorPredicate *yellowCardPredicate = new YellowCardPredicate();

public:
    CardColorReaderUseRaw();
    virtual ~CardColorReaderUseRaw();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual CardColorReaderUseRaw *generateReverseCommand() override;
    virtual colorid_t *getColorPtr();
};
#endif