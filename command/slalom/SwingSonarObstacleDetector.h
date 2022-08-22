#ifndef SwingSonarObstacleDetector_H
#define SwingSonarObstacleDetector_H

#include "SwingSonarObstacleDetector.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "CommandAndPredicate.h"
#include "Predicate.h"
#include "Stopper.h"
#include "RobotAPI.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"

using namespace ev3api;

// SwingOrder
// SwingSonarObstacleDetectorのコンストラクタ引数。
// 障害物ペットボトルの検出順番
// ・CENTER_LEFT_RIGHT_CENTER: ペットボトル間を向いているところから始め、左ペットボトル、右ペットボトルの順に検出し、初期角度に向き直る
// ・CENTER_RIGHT_LEFT_CENTER: ペットボトル間を向いているところから始め、右ペットボトル、左ペットボトルの順に検出し、初期角度に向き直る
// ・CENTER_LEFT_RIGHT: ペットボトル間を向いているところから始め、左ペットボトル、右ペットボトルの順に検出する
// ・CENTER_RIGHT_LEFT: ペットボトル間を向いているところから始め、右ペットボトル、左ペットボトルの順に検出する
//
// 実方
enum SwingOrder
{
    CENTER_LEFT_RIGHT_CENTER,
    CENTER_RIGHT_LEFT_CENTER,
    CENTER_LEFT_RIGHT,
    CENTER_RIGHT_LEFT,
};

// SwingSonarObstacleDetectorState
// SwingSonarObstacleDetectorのとり得る状態
//
// 実方
enum SwingSonarObstacleDetectorState
{
    SSD_WAIT_START,
    SSD_DETECT_OBSTACLE_LEFT,
    SSD_RETURNING_LEFT,
    SSD_DETECT_OBSTACLE_RIGHT,
    SSD_RETURNING_RIGHT,
    SSD_FINISHED,
};

// 絶対値を取得する関数
float ssodAbs(float f);

// ObstacleDetector
// 難所スラローム用の2本ペットボトル距離測定インターフェース
// run()メソッドで障害物検知をする。
// これが実装されたオブジェクトはObstacleDetectRunnerに所有され、
// 障害物検知に利用される。
// （そのとき、CommandExecutor.addCommand()メソッドから追加されない。）
// （Commandインターフェースを継承している理由は、将来Commandとして使うかもしれないから）
// swingLeftは+の値を、swingRightは-の値を渡してください。
//
// 実方
class SwingSonarObstacleDetector : public ObstacleDetector
{
private:
    SwingOrder swingOrder;

    float swingLeft = 0.0;  // 左方向への最大首振り角度
    float swingRight = 0.0; // 右方向への最大首振り角度

    int targetLeft = 0;  //左障害物判定のしきい値
    int targetRight = 0; //右障害物判定のしきい値

    SwingSonarObstacleDetectorState state;
    int pwm;
    int leftObstacleDistance = -1;
    int rightObstacleDistance = -1;
    int leftObstacleAngle = 256;
    int rightObstacleAngle = 256;
    bool detectedLeftObstacleDistance = false;
    bool detectedRightObstacleDistance = false;
    bool detectedLeftObstacleAngle = false;
    bool detectedRightObstacleAngle = false;
    Stopper *stopper;

    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetectorLeft;
    RotateRobotDistanceAngleDetector *rotateRobotDistanceAngleDetectorRight;

    Predicate *rotateRobotDistanceAngleDetectorLeftPredicate;
    Predicate *rotateRobotDistanceAngleDetectorRightPredicate;

    RotateRobotUseGyroCommandAndPredicate *rotateRobotCommandAndPredicateLeft;  // 検知したら向き直るやつ左
    RotateRobotUseGyroCommandAndPredicate *rotateRobotCommandAndPredicateRight; // 検知したら向き直るやつ右

    bool initedRotateRobotDistanceAngleDetectorLeft = false;
    bool initedRotateRobotDistanceAngleDetectorRight = false;
    bool initedRotateRobotCommandAndPreicateLeft = false;
    bool initedRotateRobotCommandAndPreicateRight = false;

    bool finished = false;

    void detectLeftObstacle(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next);
    void returningLeft(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next);
    void detectRightObstacle(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next);
    void returningRight(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next);
    void printValues(RobotAPI *robotAPI);

public:
    SwingSonarObstacleDetector(SwingOrder swingOrder, int pwm, float swingLeft, float swingRight, int targetLeft, int targetRight);
    virtual ~SwingSonarObstacleDetector();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual SwingSonarObstacleDetector *generateReverseCommand() override;
    virtual bool isFinished() override;
    virtual int getLeftObstacleDistance() override;
    virtual int getRightObstacleDistance() override;
    virtual float getLeftObstacleAngle() override;
    virtual float getRightObstacleAngle() override;
    virtual bool isDetectedLeftObstacleDistance() override;
    virtual bool isDetectedRightObstacleDistance() override;
    virtual bool isDetectedLeftObstacleAngle() override;
    virtual bool isDetectedRightObstacleAngle() override;
};

#endif
