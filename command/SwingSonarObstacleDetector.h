#ifndef SwingSonarObstacleDetector_H
#define SwingSonarObstacleDetector_H

#include "SwingSonarObstacleDetector.h"

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
}

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
    int leftObstacleDistance = -1;
    int rightObstacleDistance = -1;
    bool detectedLeftObstacleDistance = false;
    bool detectedRightObstacleDistance = false;
    SwingOrder swingOrder;

public:
    SwingSonarObstacleDetector(SwingOrder swingOrder);
    ~SwingSonarObstacleDetector();
    void run() override;
    SwingSonarObstacleDetector *generateReverseCommand() override;
    int getLeftObstacleDistance() override;
    int getRightObstacleDistance() override;
    bool isDetectedLeftObstacleDistance() override;
    bool isDetectedRightObstacleDistance() override;
};

#endif
