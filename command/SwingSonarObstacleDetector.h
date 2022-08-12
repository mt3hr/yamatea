#ifndef SwingSonarObstacleDetector_H
#define SwingSonarObstacleDetector_H

#include "SwingSonarObstacleDetector.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "CommandAndPredicate.h"
#include "WheelController.h"
#include "Predicate.h"
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
    SSD_DETECT_OBSTACLE_1,
    SSD_RETURNING1,
    SSD_DETECT_OBSTACLE_2,
    SSD_RETURNING2,
    SSD_FINISHED,
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
class SwingSonarObstacleDetector : public ObstacleDetector
{
private:
    float swingLeft = 0.0;  // 左方向への最大首振り角度
    float swingRight = 0.0; // 右方向への最大首振り角度

    int targetLeft = 0;  //左障害物判定のしきい値
    int targetRight = 0; //右障害物判定のしきい値

    SwingSonarObstacleDetectorState state;
    int pwm;
    int leftObstacleDistance = -1;
    int rightObstacleDistance = -1;
    int leftObstacleAngle = -1;
    int rightObstacleAngle = -1;
    bool detectedLeftObstacleDistance = false;
    bool detectedRightObstacleDistance = false;
    bool detectedLeftObstacleAngle = false;
    bool detectedRightObstacleAngle = false;
    SwingOrder swingOrder;
    SonarSensor *sonarSensor;
    WheelController *wheelController;
    Stopper *stopper;

/*
    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetector1 = nullptr;
    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetector2 = nullptr;

    Predicate *rotateRobotDistanceAngleDetector1Predicate = nullptr;
    Predicate *rotateRobotDistanceAngleDetector2Predicate = nullptr;

    CommandAndPredicate *rotateRobotCommandAndPredicate1 = nullptr; // 検知したら向き直るやつ1回目
    CommandAndPredicate *rotateRobotCommandAndPredicate2 = nullptr; // 検知したら向き直るやつ2回目
    */

    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetector1;
    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetector2;

    Predicate *rotateRobotDistanceAngleDetector1Predicate;
    Predicate *rotateRobotDistanceAngleDetector2Predicate;

    CommandAndPredicate *rotateRobotCommandAndPredicate1; // 検知したら向き直るやつ1回目
    CommandAndPredicate *rotateRobotCommandAndPredicate2; // 検知したら向き直るやつ2回目

    bool initedRotateRobotDistanceAngleDetector1 = false;
    bool initedRotateRobotDistanceAngleDetector2 = false;
    bool initedRotateRobotCommandAndPreicate1 = false;
    bool initedRotateRobotCommandAndPreicate2 = false;

    bool finished = false;

public:
    SwingSonarObstacleDetector(SwingOrder swingOrder, int pwm, float swingLeft, float swingRight, int targetLeft, int targetRight, SonarSensor *sonarSensor, WheelController *WheelController);
    ~SwingSonarObstacleDetector();
    void run() override;
    SwingSonarObstacleDetector *generateReverseCommand() override;
    bool isFinished() override;
    int getLeftObstacleDistance() override;
    int getRightObstacleDistance() override;
    float getLeftObstacleAngle() override;
    float getRightObstacleAngle() override;
    bool isDetectedLeftObstacleDistance() override;
    bool isDetectedRightObstacleDistance() override;
    bool isDetectedLeftObstacleAngle() override;
    bool isDetectedRightObstacleAngle() override;
};

#endif
