#include "UFORunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"
#include "math.h"
#include "DistancePredicate.h"
#include "RotateRobot.h"
#include "CommandAndPredicate.h"

using namespace std;
using namespace ev3api;

UFORunner::UFORunner(float na, int wp, int rp, WheelController *wc, SonarSensor *ss, ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
    state = DETECTING_OBSTACLE;
    wheelController = wc;
    sonarSensor = ss;
    n = n;
    walkerPow = wp;
    rotatePow = rp;
}

UFORunner::~UFORunner()
{
    delete p_nWalker;
    delete n_xdivide2Walker;
    delete turnNCommand;
    delete p_nDistancePredicate;
    delete n_xdivide2DistancePreicate;
    delete turnNPredicate;
}

void UFORunner::run()
{
    ObstacleDetector *obstacleDetector = getObstacleDetector();
    switch (state)
    {
    case DETECTING_OBSTACLE: // 障害物検出状態
    {
        obstacleDetector->run();

        if (obstacleDetector->isDetectedLeftObstacleDistance() && obstacleDetector->isDetectedRightObstacleDistance())
        {
            state = CALCRATING;
        }

        if (state != CALCRATING)
        {
            break;
        }
    }

    case CALCRATING: // 角度計算処理。各値については要求モデルを参照して。
    {
        // 計算処理が複数回呼び出されないように
        if (startedCalcrate)
        {
            break;
        }
        startedCalcrate = true;

        float ik = obstacleDetector->getLeftObstacleDistance();
        float dk = obstacleDetector->getRightObstacleDistance();
        float p = obstacleDetector->getObstacleAngle();

        // C++のmathの三角関数系統はラジアンをうけとるしラジアンを返してくる

        // １，余弦定理で障害物間の距離Xを求める
        // X^2=Ik^2+Dk^2-(2Ik×Dk×cos∠P)
        float xSquare = pow(ik, 2.0) + pow(dk, 2.0) - ((2 * ik) * dk * (cos(p * (M_PI / 180) * (M_PI / 180))));
        float x = pow(xSquare, 1 / 2);

        // ２，Xの中点から車体側に対し垂直に距離Nを引き、NIを求める
        // X/2^2+N^2=NI^2
        float niSquare = pow(x / 2, 2) + pow(n, 2);
        float ni = pow(niSquare, 1 / 2);

        // 3,∠Iから∠NID（arcsin((x/2)/NI)）を引き余弦定理で距離PNを求める
        // PN^2=Ik^2+NI^2-(2Ik×NI×cos∠PIN)
        //・∠pinの求め方が分からない。
        // A.∠I-∠NID
        float pi = ik;
        float pd = dk;
        float pin = acos((pow(pi, 2) + pow(x, 2) - pow(pd, 2)) / (2 * pi * x)) - acos((pow(x / 2, 2) + pow(ni, 2) - pow(n, 2)) / (2 * x / 2 * ni));
        float pnSquare = pow(ik, 2) + pow(ni, 2) - ((2 * ik) * ni * (cos(pin * (M_PI / 180)) * (M_PI / 180)));
        pn = pow(pnSquare, 1 / 2);

        // 4,cos∠IPNをもとめ、逆関数で角度求める。
        // ∠IPN=arcsin(∠IPN)
        ipn = asin(ipn * (M_PI / 180)) * (M_PI / 180);

        state = RUNNING_P_N;

        if (state != RUNNING_P_N)
        {
            break;
        }
    }

    case RUNNING_P_N: // PからNまでの走行
    {
        if (!initedP_N)
        {
            initedP_N = true;
            p_nWalker = new Walker(walkerPow, walkerPow, wheelController);
            p_nDistancePredicate = new DistancePredicate(pn, wheelController->getLeftWheel());
            p_nDistancePredicate->preparation();
        }

        p_nWalker->run();

        if (p_nDistancePredicate->test())
        {
            state = TURNNING_N;
        }

        if (state != TURNNING_N)
        {
            break;
        }
    }

    case TURNNING_N: // N地点での旋回
    {
        if (!initedTurnN)
        {
            initedTurnN = true;
            CommandAndPredicate *commandAndPredicate = generateRotateRobotCommand(ipn, rotatePow, wheelController);
            turnNCommand = commandAndPredicate->getCommand();
            turnNPredicate = commandAndPredicate->getPredicate();
        }

        turnNCommand->run();

        if (turnNPredicate->test())
        {
            state = RUNNING_N_XDIVIDE2;
        }

        if (state != RUNNING_N_XDIVIDE2)
        {
            break;
        }
    }

    case RUNNING_N_XDIVIDE2: // Nから2/xまでの走行
    {
        if (!initedN_XDivide2)
        {
            initedN_XDivide2 = true;
            n_xdivide2Walker = new Walker(walkerPow, walkerPow, wheelController);
            n_xdivide2DistancePreicate = new DistancePredicate(pn, wheelController->getLeftWheel());
            n_xdivide2DistancePreicate->preparation();
        }

        n_xdivide2Walker->run();

        if (n_xdivide2DistancePreicate->test())
        {
            state = FINISHED;
        }

        if (state != FINISHED)
        {
            break;
        }
    }

    case FINISHED:
    {
        break;
    }

    default:
    {
        break;
    }
    }

    return;
}

UFORunner *UFORunner::generateReverseCommand()
{
    return new UFORunner(n, walkerPow, rotatePow, wheelController, sonarSensor, getObstacleDetector()->generateReverseCommand());
}

bool UFORunner::isFinished()
{
    return state == FINISHED;
}