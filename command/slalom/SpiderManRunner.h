#ifndef SpiderManRunner_H
#define SpiderManRunner_H

#include "ObstacleDetectRunner.h"
#include "FinishConfirmable.h"
#include "Walker.h"

enum SpiderManState
{
    SPDM_WAIT_START,
    SPDM_DETECT_OBSTACLE_LEFT_CENTER,
    SPDM_DETECT_OBSTACLE_CENTER_RIGHT,
    SPDM_CALCRATING,
    SPDM_TURNNING_IPN_AT_P,
    SPDM_TURNNING_DPN_AT_P,
    SPDM_WALKING_P_N,
    SPDM_WALKING_XDIVIDE2_AT_N,
    SPDM_FINISH,
};

class SpiderManRunner : public ObstacleDetectRunner, public FinishConfirmable
{
private:
    SpiderManState state = SPDM_WAIT_START;
    bool reverse = false;                               // コンストラクタから初期化

    int pwm;
    float angleLeftCenter;
    float angleCenterRight;
    int targetLeft;
    int targetCenter;
    int targetRight;

    int currentAngle;
    int currentDistance;
    int preAngle;
    int preDistance;

public:
    SpiderManRunner(int pwm, float angleLeftCenter, float angleCenterRight, int targetLeft, int targetCenter, int targetRight);
    virtual ~SpiderManRunner();
    virtual void run(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual SpiderManRunner *generateReverseCommand();
    virtual bool isFinished();
};

#endif