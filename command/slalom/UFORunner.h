#ifndef UFORunner_H
#define UFORunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "FinishConfirmable.h"
#include "WheelDistancePredicate.h"
#include "Predicate.h"
#include "Walker.h"
#include "Command.h"
#include "Stopper.h"

// UFOBehavior
// UFO走行のペットボトル検出時の挙動
// 
// 実方
enum UFOBehavior
{
    SWING_SONAR, // 障害物間をむいているところからはじめ、右を向き、左を向いて検出したら終わる。
    CLOCKWISE,   // 左障害物間を向いているところから始め、右に向いて検出したら終わる。
};

// UFORunnerState
// UFORunnerのとり得る状態
//
// 実方
enum UFORunnerState
{
    UFO_DETECTING_OBSTACLE,
    UFO_CALCRATING,
    UFO_TURNNIN_TO_P,
    UFO_TURNNING_P_IPN,
    UFO_TURNNING_P_DPN,
    UFO_RUNNING_P_N,
    UFO_TURNNING_N,
    UFO_RUNNING_N_XDIVIDE2,
    UFO_FINISHED,
};

// 度をラジアンに変換する関数
float toRadian(float degree);
// ラジアンを度に変換する関数
float toDegree(float radian);

// UFORunner
// UFO走行をするクラス。
// UFO走行については要求モデルを参照
// いずれかのinitializeメソッドを呼び出してください。
//
// 実方
class UFORunner : public ObstacleDetectRunner, public FinishConfirmable
{
private:
    UFORunnerState state;
    UFOBehavior behavior;
    bool reverse = false;

    bool initedObstacleDetector = false;

    Command *turnToPCommand;
    Command *turnPIPNCommand;
    Command *turnPDPNCommand;
    Walker *p_nWalker;
    Walker *n_xdivide2Walker;
    Command *turnNCommand;

    Predicate *turnToPPredicate;
    Predicate *turnPIPNPredicate;
    Predicate *turnPDPNPredicate;
    WheelDistancePredicate *p_nDistancePredicate;
    WheelDistancePredicate *n_xdivide2DistancePreicate;
    Predicate *turnNPredicate;

    bool initedTurnToP = false;
    bool initedTurnPIPN = false;
    bool initedTurnPDPN = false;
    bool startedCalcrate = false;
    bool initedP_N = false;
    bool initedN_XDivide2 = false;
    bool initedTurnN = false;

    float ik;
    float dk;
    float p;
    float n;
    float x;
    float ni;
    float pn;
    float i;
    float din;
    float ipn;
    float dpn;
    float pin;
    float nTurnAngle;

    int walkerPow;
    int rotatePow;

    Stopper *stopper;

    // generateReverseCommandやコンストラクタで使われる
    UFORunner(float n, int walkerPow, int rotatePow);

public:
    // 障害物間からから距離角度計測を始めるもの
    UFORunner(float na, int wp, int rp, float swingLeftAngle, float swingRightAngle, int targetLeftDistance, int targetRightDistance);
    // 左障害物を向いている状態から距離角度計測を始めるもの
    UFORunner(float na, int wp, int rp, float angle, int targetLeft, int targetRight, int skipFrameAfterDetectFirstObstacle, bool facingObstacle);
    virtual ~UFORunner();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual UFORunner *generateReverseCommand() override;
    virtual bool isFinished() override;
};

#endif