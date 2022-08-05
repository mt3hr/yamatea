#ifndef Handler_H
#define Handler_H

// Handle
// PIDTargetBrightnessCalibratorで使われるHandleのインターフェース。
// 各キャリブレータで値の読み取りが完了したあとに（例えばPIDTracer.targetBrightnessなどの）値を更新するために使われます。
// オーバーライドして使ってください。
// 既知のHandler: SetPIDTargetBrightnessWhenCalibratedHandler
class Handler
{
private:
public:
    virtual ~Handler();
    virtual void handle();
};

#endif