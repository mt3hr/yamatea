#include "UFORunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "SonarSensor.h"
#include "math.h"
#include "DistancePredicate.h"
#include "CommandAndPredicate.h"
#include "Setting.h"
#include "string"
#include "DebugUtil.h"
#include "RobotAPI.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "SwingSonarObstacleDetector.h"

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

UFORunner::UFORunner(float na, int wp, int rp, float sl, float sr, int tl, int tr) : ObstacleDetectRunner(new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT, rp, sl, sr, tl, tr))
{
    state = UFO_DETECTING_OBSTACLE;
    n = na;
    walkerPow = wp;
    rotatePow = rp;
    this->swingLeftAngle = sl;
    this->swingRightAngle = sr;
    this->targetLeftDistance = tl;
    this->targetRightDistance = tr;
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

        writeDebug("UFO_DETECTING_OBSTACLE finished");
        flushDebug(INFO, robotAPI);
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
        pn = sqrt(pow(ni, 2) + pow(ik, 2) - 2 * ik * ni * cos(toRadian(pin)));

        // 4,cos∠IPNをもとめ、逆関数で角度求める。
        // ∠IPN=arcsin(∠IPN)
        ipn = toDegree(acos((pow(pn, 2) + pow(ik, 2) - pow(ni, 2)) / (2 * pn * ik)));
        dpn = p - ipn;

        nTurnAngle = 180 - (180 - pin - ipn) - (180 - 90 - din);

        if (!reverse)
        {
            state = UFO_TURNNING_P_IPN;
        }
        else
        {
            state = UFO_TURNNING_P_DPN;
        }

        if (!(state == UFO_TURNNIN_TO_P || state == UFO_TURNNING_P_IPN || state == UFO_TURNNING_P_DPN))
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
        writeDebug("pin: ");
        writeDebug(pin);
        writeEndLineDebug();
        writeDebug("din: ");
        writeDebug(din);
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
        flushDebug(DEBUG, robotAPI);

        writeDebug("UFO_CALCRATING finished");
        flushDebug(INFO, robotAPI);
    }

    case UFO_TURNNIN_TO_P: // I方向を向くように旋回
    {
        if (!initedTurnToP)
        {
            CommandAndPredicate *turnToPCommandAndPredicate = new RotateRobotUseGyroCommandAndPredicate(obstacleDetector->getLeftObstacleAngle(), rotatePow, robotAPI);
            turnToPCommand = turnToPCommandAndPredicate->getCommand();
            turnToPPredicate = turnToPCommandAndPredicate->getPredicate();
            turnToPCommandAndPredicate->getPredicate()->preparation(robotAPI);
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
        flushDebug(INFO, robotAPI);
    }

    case UFO_TURNNING_P_IPN: // IPN右回転
    {
        if (!initedTurnPIPN)
        {
            CommandAndPredicate *turnPIPNCommandAndPredicate = new RotateRobotUseGyroCommandAndPredicate(-ipn, rotatePow, robotAPI);
            turnPIPNCommand = turnPIPNCommandAndPredicate->getCommand();
            turnPIPNPredicate = turnPIPNCommandAndPredicate->getPredicate();
            turnPIPNCommandAndPredicate->getPredicate()->preparation(robotAPI);
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
        flushDebug(INFO, robotAPI);
    }

    case UFO_TURNNING_P_DPN: // DPN右回転
    {
        if (!initedTurnPDPN)
        {
            CommandAndPredicate *turnPDPNCommandAndPredicate = new RotateRobotUseGyroCommandAndPredicate(-dpn, rotatePow, robotAPI);
            turnPDPNCommand = turnPDPNCommandAndPredicate->getCommand();
            turnPDPNPredicate = turnPDPNCommandAndPredicate->getPredicate();
            turnPDPNCommandAndPredicate->getPredicate()->preparation(robotAPI);
            initedTurnPDPN = true;
        }

        turnPDPNCommand->run(robotAPI);

        if (turnPDPNPredicate->test(robotAPI))
        {
            state = UFO_RUNNING_P_N;
        }

        if (state != UFO_RUNNING_P_N)
        {
            break;
        }

        writeDebug("UFO_TURNNING_P_DPN finished");
        flushDebug(INFO, robotAPI);
    }

    case UFO_RUNNING_P_N: // PからNまでの走行
    {
        if (!initedP_N)
        {
            p_nWalker = new Walker(walkerPow, walkerPow);
            p_nDistancePredicate = new DistancePredicate(pn, robotAPI->getLeftWheel());
            p_nDistancePredicate->preparation(robotAPI);
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
        flushDebug(INFO, robotAPI);
    }

    case UFO_TURNNING_N: // N地点での旋回
    {
        if (!initedTurnN)
        {
            CommandAndPredicate *commandAndPredicate = new RotateRobotUseGyroCommandAndPredicate(-nTurnAngle, rotatePow, robotAPI);
            turnNCommand = commandAndPredicate->getCommand();
            turnNPredicate = commandAndPredicate->getPredicate();
            commandAndPredicate->getPredicate()->preparation(robotAPI);
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
        flushDebug(INFO, robotAPI);
    }

    case UFO_RUNNING_N_XDIVIDE2: // Nから2/xまでの走行
    {
        if (!initedN_XDivide2)
        {
            n_xdivide2Walker = new Walker(walkerPow, walkerPow);
            n_xdivide2DistancePreicate = new DistancePredicate(n, robotAPI->getLeftWheel());
            n_xdivide2DistancePreicate->preparation(robotAPI);
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
        flushDebug(INFO, robotAPI);
    }

    case UFO_FINISHED:
    {
        writeDebug("UFO_FINISHED finished");
        flushDebug(INFO, robotAPI);
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
    UFORunner *reversed = new UFORunner(n, walkerPow, rotatePow, swingRightAngle, swingLeftAngle, targetRightDistance, targetLeftDistance);
    reversed->reverse = !reverse;
    reversed->setObstacleDetector(getObstacleDetector()->generateReverseCommand());
    return reversed;
}

bool UFORunner::isFinished()
{
    return state == UFO_FINISHED;
}
