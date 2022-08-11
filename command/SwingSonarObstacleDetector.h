#ifndef SwingSonarObstacleDetector_H
#define SwingSonarObstacleDetector_H

#include "SwingSonarObstacleDetector.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "CommandAndPredicate.h"
#include "WheelController.h"
#include "Predicate.h"
#include "FinishConfirmable.h"
#include "Stopper.h"

using namespace ev3api;

// SwingOrder
// SwingSonarObstacleDetectorのコンストラクタ引数。
// 障害物ペットボトルの検出順番
// ・CENTER_LEFT_RIGHT: ペットボトル間を向いているところからはじめ、左ペットボトル、右ペットボトルの順に検出する
// ・CENTER_RIGHT_LEFT: ペットボトル間を向いているところから始め、右ペットボトル、左ペットボトルの順に検出する
// ・LEFT_RIGHT: 左ペットボトルを向いている状態からはじめ、右を検出する
// ・RIGHT_LEFT: 右ペットボトルを向いている状態からはじめ、左を検出する
//
// 実方
enum SwingOrder
{
    CENTER_LEFT_RIGHT,
    CENTER_RIGHT_LEFT,
    LEFT_RIGHT,
    RIGHT_LEFT,
};

enum SwingSonarObstacleDetectorState
{
    DETECT_OBSTACLE_1,
    RETURNING1,
    DETECT_OBSTACLE_2,
    RETURNING2,
    FINISHED,
};

// ObstacleDetector
// 難所スラローム用の2本ペットボトル距離測定インターフェース
// run()メソッドで障害物検知をする。
// これが実装されたオブジェクトはObstacleDetectRunnerに所有され、
// 障害物検知に利用される。
// そのとき、CommandExecutor.addCommand()メソッドから追加されない。
// （Commandインターフェースを継承している理由は、将来Commandとして使うかもしれないから）
//
// 実方
class SwingSonarObstacleDetector : public ObstacleDetector,
                                   public FinishConfirmable
{
private:
    SwingSonarObstacleDetectorState state;
    int pwm;
    int leftObstacleDistance = -1;
    int rightObstacleDistance = -1;
    int obstacleAngle = -1;
    bool detectedLeftObstacleDistance = false;
    bool detectedRightObstacleDistance = false;
    bool detectedObstacleAngle = false;
    SwingOrder swingOrder;
    SonarSensor *sonarSensor;
    WheelController *wheelController;
    Stopper *stopper;

    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetector1 = nullptr;
    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetector2 = nullptr;

    Predicate *rotateRobotDistanceAngleDetector1Predicate = nullptr;
    Predicate *rotateRobotDistanceAngleDetector2Predicate = nullptr;

    CommandAndPredicate *rotateRobotCommandAndPredicate1 = nullptr; // 検知したら向き直るやつ1回目
    CommandAndPredicate *rotateRobotCommandAndPredicate2 = nullptr; // 検知したら向き直るやつ2回目

    bool initedRotateRobotDistanceAngleDetector1 = false;
    bool initedRotateRobotDistanceAngleDetector2 = false;
    bool initedRotateRobotCommandAndPreicate1 = false;
    bool initedRotateRobotCommandAndPreicate2 = false;

    bool finished = false;

public:
    SwingSonarObstacleDetector(SwingOrder swingOrder, int pwm, SonarSensor *sonarSensor, WheelController *WheelController);
    ~SwingSonarObstacleDetector();
    void run() override;
    SwingSonarObstacleDetector *generateReverseCommand() override;
    bool isFinished() override;
    int getLeftObstacleDistance() override;
    int getRightObstacleDistance() override;
    float getObstacleAngle() override;
    bool isDetectedLeftObstacleDistance() override;
    bool isDetectedRightObstacleDistance() override;
    bool isDetectedObstacleAngle() override;
};

#endif
