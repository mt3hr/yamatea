#include "UFORunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "math.h"
#include "DistancePredicate.h"
#include "RotateRobot.h"
#include "CommandAndPredicate.h"
#include "Setting.h"
#include "string"
#include "DebugUtil.h"
#include "RobotAPI.h"

using namespace std;
using namespace ev3api;

float toRadian(float degree)
{
    return degree * M_PI / 180;
}

float toDegree(float radian)
{
    return radian * 180 / M_PI;
}

// TODO 右コースに対応で規定なさそう
UFORunner::UFORunner(float na, int wp, int rp, ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
    state = UFO_DETECTING_OBSTACLE;
    n = na;
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

void UFORunner::run(RobotAPI *robotAPI)
{
    ObstacleDetector *obstacleDetector = getObstacleDetector();
    switch (state)
    {
    case UFO_DETECTING_OBSTACLE: // 障害物検出状態
    {
        obstacleDetector->run(robotAPI);

        if (obstacleDetector->isFinished())
        {
            state = UFO_CALCRATING;
        }

        if (state != UFO_CALCRATING)
        {
            break;
        }
    }

    case UFO_CALCRATING: // 角度計算処理。各値については要求モデルを参照して。
    {
        // 計算処理が複数回呼び出されないように
        if (startedCalcrate)
        {
            break;
        }
        startedCalcrate = true;

        ik = obstacleDetector->getLeftObstacleDistance() + distanceFromSonarSensorToAxle;
        dk = obstacleDetector->getRightObstacleDistance() + distanceFromSonarSensorToAxle;
        p = obstacleDetector->getLeftObstacleAngle() + obstacleDetector->getRightObstacleAngle();

        // C++のmathの三角関数系統はラジアンをうけとるしラジアンを返してくる

        // １，余弦定理で障害物間の距離Xを求める
        // X^2=Ik^2+Dk^2-(2Ik×Dk×cos∠P)
        x = sqrt(pow(ik, 2) + pow(dk, 2) - 2 * ik * dk * cos(toRadian(p)));

        // ２，Xの中点から車体側に対し垂直に距離Nを引き、NIを求める
        // X/2^2+N^2=NI^2
        ni = sqrt(pow(x / 2, 2) + pow(n, 2));

        // 3,∠Iから∠NID（arcsin((x/2)/NI)）を引き余弦定理で距離PNを求める
        // PN^2=Ik^2+NI^2-(2Ik×NI×cos∠PIN)
        i = toDegree(acos((pow(ik, 2) + pow(x, 2) - pow(dk, 2)) / (2 * ik * x)));
        din = toDegree(acos((pow(x / 2, 2) + pow(ni, 2) - pow(n, 2)) / (2 * x / 2 * ni)));
        pin = i - din;
        pn = sqrt(pow(ni, 2) + pow(ik, 2) - 2 * ik * ni * (cos(toRadian(pin))));

        // 4,cos∠IPNをもとめ、逆関数で角度求める。
        // ∠IPN=arcsin(∠IPN)
        ipn = toDegree(acos(((pow(pn, 2) + pow(ik, 2) - pow(ni, 2)) / (2 * pn * ik))));

        nTurnAngle = 180 - (180 - pin - ipn) - toDegree(asin(n / ni));

        state = UFO_TURNNIN_TO_P;

        if (state != UFO_TURNNIN_TO_P)
        {
            break;
        }

        writeDebug("ik: ");
        writeDebug(ik);
        writeEndLineDebug();
        writeDebug("dk: ");
        writeDebug(dk);
        writeEndLineDebug();
        writeDebug("p: ");
        writeDebug(p);
        writeEndLineDebug();
        writeDebug("x: ");
        writeDebug(x);
        writeEndLineDebug();
        writeDebug("ni: ");
        writeDebug(ni);
        writeEndLineDebug();
        writeDebug("i: ");
        writeDebug(i);
        writeEndLineDebug();
        writeDebug("din: ");
        writeDebug(din);
        writeEndLineDebug();
        writeDebug("pin: ");
        writeDebug(pin);
        writeEndLineDebug();
        writeDebug("pn: ");
        writeDebug(pn);
        writeEndLineDebug();
        writeDebug("ipn: ");
        writeDebug(ipn);
        writeEndLineDebug();
        writeDebug("nTurnAngle: ");
        writeDebug(nTurnAngle);
        writeEndLineDebug();
        flushDebug(D, robotAPI);

        writeDebug("UFO_CALCRATING finished");
        flushDebug(I, robotAPI);
    }

    case UFO_TURNNIN_TO_P: // I方向を向くように旋回
    {
        if (!initedTurnToP)
        {
            CommandAndPredicate *turnToPCommandAndPredicate = generateRotateRobotCommand(obstacleDetector->getLeftObstacleAngle(), rotatePow, robotAPI);
            turnToPCommand = turnToPCommandAndPredicate->getCommand();
            turnToPPredicate = turnToPCommandAndPredicate->getPredicate();
            turnToPCommandAndPredicate->getPreHandler()->handle(robotAPI);
            initedTurnToP = true;
        }

        turnToPCommand->run(robotAPI);

        if (turnToPPredicate->test(robotAPI))
        {
            state = UFO_TURNNING_P_IPN;
        }

        if (state != UFO_TURNNING_P_IPN)
        {
            break;
        }

        writeDebug("UFO_TURNNIN_TO_P finished");
        flushDebug(I, robotAPI);
    }

    case UFO_TURNNING_P_IPN: // IPN右回転
    {
        if (!initedTurnPIPN)
        {
            CommandAndPredicate *turnPIPNCommandAndPredicate = generateRotateRobotCommand(-ipn, rotatePow, robotAPI);
            turnPIPNCommand = turnPIPNCommandAndPredicate->getCommand();
            turnPIPNPredicate = turnPIPNCommandAndPredicate->getPredicate();
            turnPIPNCommandAndPredicate->getPreHandler()->handle(robotAPI);
            initedTurnPIPN = true;
        }

        turnPIPNCommand->run(robotAPI);

        if (turnPIPNPredicate->test(robotAPI))
        {
            state = UFO_RUNNING_P_N;
        }

        if (state != UFO_RUNNING_P_N)
        {
            break;
        }

        writeDebug("UFO_TURNNING_P_IPN finished");
        flushDebug(I, robotAPI);
    }

    case UFO_RUNNING_P_N: // PからNまでの走行
    {
        if (!initedP_N)
        {
            p_nWalker = new Walker(walkerPow, walkerPow);
            p_nDistancePredicate = new DistancePredicate(pn, robotAPI->getLeftWheel());
            p_nDistancePredicate->preparation();
            initedP_N = true;
        }

        p_nWalker->run(robotAPI);

        if (p_nDistancePredicate->test(robotAPI))
        {
            state = UFO_TURNNING_N;
        }

        if (state != UFO_TURNNING_N)
        {
            break;
        }

        writeDebug("UFO_RUNNING_P_N finished");
        flushDebug(I, robotAPI);
    }

    case UFO_TURNNING_N: // N地点での旋回
    {
        if (!initedTurnN)
        {
            CommandAndPredicate *commandAndPredicate = generateRotateRobotCommand(-nTurnAngle, rotatePow, robotAPI);
            turnNCommand = commandAndPredicate->getCommand();
            turnNPredicate = commandAndPredicate->getPredicate();
            commandAndPredicate->getPreHandler()->handle(robotAPI);
            initedTurnN = true;
        }

        turnNCommand->run(robotAPI);

        if (turnNPredicate->test(robotAPI))
        {
            state = UFO_RUNNING_N_XDIVIDE2;
        }

        if (state != UFO_RUNNING_N_XDIVIDE2)
        {
            break;
        }

        writeDebug("UFO_TURNNING_N finished");
        flushDebug(I, robotAPI);
    }

    case UFO_RUNNING_N_XDIVIDE2: // Nから2/xまでの走行
    {
        if (!initedN_XDivide2)
        {
            n_xdivide2Walker = new Walker(walkerPow, walkerPow);
            n_xdivide2DistancePreicate = new DistancePredicate(pn, robotAPI->getLeftWheel());
            n_xdivide2DistancePreicate->preparation();
            initedN_XDivide2 = true;
        }

        n_xdivide2Walker->run(robotAPI);

        if (n_xdivide2DistancePreicate->test(robotAPI))
        {
            state = UFO_FINISHED;
        }

        if (state != UFO_FINISHED)
        {
            break;
        }

        writeDebug("UFO_RUNNING_N_XDIVIDE2 finished");
        flushDebug(I, robotAPI);
    }

    case UFO_FINISHED:
    {
        writeDebug("UFO_FINISHED finished");
        flushDebug(I, robotAPI);
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
    return new UFORunner(n, walkerPow, rotatePow, getObstacleDetector()->generateReverseCommand());
}

bool UFORunner::isFinished()
{
    return state == UFO_FINISHED;
}
