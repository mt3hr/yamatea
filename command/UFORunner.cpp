#include "UFORunner.h"
#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"
#include "math.h"
#include "DistancePredicate.h"
#include "RotateRobot.h"
#include "CommandAndPredicate.h"
#include "Setting.h"
#include "string"         //TODO 消して
#include "sstream"        //TODO 消して
#include "vector"         //TODO 消して
#include "PrintMessage.h" //TODO 消して

using namespace std;
using namespace ev3api;

// TODO 右コースに対応で規定なさそう
UFORunner::UFORunner(float na, int wp, int rp, WheelController *wc, SonarSensor *ss, ObstacleDetector *obstacleDetector) : ObstacleDetectRunner(obstacleDetector)
{
    state = UFO_DETECTING_OBSTACLE;
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
    case UFO_DETECTING_OBSTACLE: // 障害物検出状態
    {
        obstacleDetector->run();

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
        // x = float(sqrt(ik * ik + dk * dk - ((2 * ik) * dk * (cos(p * (M_PI / 180)) * (180 / M_PI)))));
        x = float(sqrt((ik * ik) + (dk * dk) - (2 * ik * dk * (cos(p * (M_PI / 180)) * (180 / M_PI)))));

        // ２，Xの中点から車体側に対し垂直に距離Nを引き、NIを求める
        // X/2^2+N^2=NI^2
        ni = float(sqrt(x / 2 * x / 2 + n * n));

        // 3,∠Iから∠NID（arcsin((x/2)/NI)）を引き余弦定理で距離PNを求める
        // PN^2=Ik^2+NI^2-(2Ik×NI×cos∠PIN)
        //・∠pinの求め方が分からない。
        // A.∠I-∠NID
        // pn = float(sqrt(ik * ik + ni * ni - ((2 * ik) * ni * (cos(pin * (M_PI / 180)) * (180 / M_PI)))));
        pin = float(acos(((ik * ik + x * x - dk * dk) / (2 * ik * x)) * (M_PI / 180)) * (180 / M_PI) - acos(((x / 2 * x / 2 + ni * 2 - n * n) / (2 * (x / 2) * ni)) * (M_PI / 180)) * (180 / M_PI));
        pn = sqrt(ni * ni + ik * ik - 2 * ik * ni * (cos(pin * (M_PI / 180) * 180 / M_PI)));

        // 4,cos∠IPNをもとめ、逆関数で角度求める。
        // ∠IPN=arcsin(∠IPN)
        // ipn = float(asin(ipn * (M_PI / 180)) * (180 / M_PI));
        ipn = acos(((pn * pn + ik * ik - ni * ni) / (2 * pn * ik)) * (M_PI / 180)) * (180 / M_PI);

        nTurnAngle = float(acos((((n * n) + (ni * ni) - ((x / 2) * (x / 2)) / (2 * n * ni)) * (M_PI / 180))) * (180 / M_PI));

        state = UFO_TURNNIN_TO_P;

        if (state != UFO_TURNNIN_TO_P)
        {
            break;
        }

        //表示しちゃお
        // 実機で出力しまくるとなんか落ちる
        stringstream pins;                                                                                             // TODO 消して
        stringstream iks;                                                                                              // TODO 消して
        stringstream dks;                                                                                              // TODO 消して
        stringstream ipns;                                                                                             // TODO 消して
        stringstream ntas;                                                                                             // TODO 消して
        stringstream xs;                                                                                               // TODO 消して
        stringstream ps;                                                                                               // TODO 消して
        stringstream nis;                                                                                              // TODO 消して
        stringstream pns;                                                                                              // TODO 消して
        stringstream ntaas;                                                                                            // TODO 消して
        pins.clear();                                                                                                  // TODO 消して
        iks.clear();                                                                                                   // TODO 消して
        dks.clear();                                                                                                   // TODO 消して
        ipns.clear();                                                                                                  // TODO 消して
        ntas.clear();                                                                                                  // TODO 消して
        xs.clear();                                                                                                    // TODO 消して
        ps.clear();                                                                                                    // TODO 消して
        nis.clear();                                                                                                   // TODO 消して
        pns.clear();                                                                                                   // TODO 消して
        ntaas.clear();                                                                                                 // TODO 消して
        pins.str("");                                                                                                  // TODO 消して
        iks.str("");                                                                                                   // TODO 消して
        dks.str("");                                                                                                   // TODO 消して
        ipns.str("");                                                                                                  // TODO 消して
        ntas.str("");                                                                                                  // TODO 消して
        xs.str("");                                                                                                    // TODO 消して
        ps.str("");                                                                                                    // TODO 消して
        nis.str("");                                                                                                   // TODO 消して
        pns.str("");                                                                                                   // TODO 消して
        ntaas.str("");                                                                                                 // TODO 消して
        iks << "ik: " << ik;                                                                                           // TODO 消して
        dks << "dk: " << dk;                                                                                           // TODO 消して
        pins << "pin: " << pin;                                                                                        // TODO 消して
        ipns << "ipn: " << ipn;                                                                                        // TODO 消して
        ntas << "nTurnAngle: " << nTurnAngle;                                                                          // TODO 消して
        xs << "x: " << x;                                                                                              // TODO 消して
        ps << "p: " << p;                                                                                              // TODO 消して
        nis << "ni: " << ni;                                                                                           // TODO 消して
        pns << "pn: " << pn;                                                                                           // TODO 消して
        ntaas << "nTurnAngleAcosArg: " << (((n * n) + (ni * ni) - ((x / 2) * (x / 2)) / (2 * n * ni)) * (M_PI / 180)); // TODO 消して
        vector<string> messageLines;                                                                                   // TODO 消して
        messageLines.push_back(iks.str());                                                                              // TODO 消して
        messageLines.push_back(dks.str());                                                                              // TODO 消して
        messageLines.push_back(ps.str());                                                                              // TODO 消して
        messageLines.push_back(xs.str());                                                                              // TODO 消して
        messageLines.push_back(pins.str());                                                                            // TODO 消して
        messageLines.push_back(pns.str());                                                                             // TODO 消して
        messageLines.push_back(ipns.str());                                                                            // TODO 消して
        messageLines.push_back(nis.str());                                                                             // TODO 消して
        messageLines.push_back(ntas.str());                                                                            // TODO 消して
        messageLines.push_back(ntaas.str());                                                                           // TODO 消して
        PrintMessage *resultPrintCommand = new PrintMessage(messageLines, true);                                       // TODO 消して
        resultPrintCommand->run();                                                                                     // TODO 消して
        delete resultPrintCommand;                                                                                     // TODO 消して
    }

    case UFO_TURNNIN_TO_P: // I方向を向くように旋回
    {
        if (!initedTurnToP)
        {
            CommandAndPredicate *turnToPCommandAndPredicate = generateRotateRobotCommand(obstacleDetector->getLeftObstacleAngle(), rotatePow, wheelController);
            turnToPCommand = turnToPCommandAndPredicate->getCommand();
            turnToPPredicate = turnToPCommandAndPredicate->getPredicate();
            turnToPCommandAndPredicate->getPreHandler()->handle();
            initedTurnToP = true;
        }

        turnToPCommand->run();

        if (turnToPPredicate->test())
        {
            state = UFO_TURNNING_P_IPN;
        }

        if (state != UFO_TURNNING_P_IPN)
        {
            break;
        }
    }

    case UFO_TURNNING_P_IPN: // IPN右回転
    {
        if (!initedTurnPIPN)
        {
            CommandAndPredicate *turnPIPNCommandAndPredicate = generateRotateRobotCommand(ipn, rotatePow, wheelController);
            turnPIPNCommand = turnPIPNCommandAndPredicate->getCommand();
            turnPIPNPredicate = turnPIPNCommandAndPredicate->getPredicate();
            turnPIPNCommandAndPredicate->getPreHandler()->handle();
        }

        turnPIPNCommand->run();

        if (turnPIPNPredicate->test())
        {
            state = UFO_RUNNING_P_N;
        }

        if (state != UFO_RUNNING_P_N)
        {
            break;
        }
    }

    case UFO_RUNNING_P_N: // PからNまでの走行
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
            state = UFO_TURNNING_N;
        }

        if (state != UFO_TURNNING_N)
        {
            break;
        }
    }

    case UFO_TURNNING_N: // N地点での旋回
    {
        if (!initedTurnN)
        {
            initedTurnN = true;
            CommandAndPredicate *commandAndPredicate = generateRotateRobotCommand(nTurnAngle, rotatePow, wheelController);
            turnNCommand = commandAndPredicate->getCommand();
            turnNPredicate = commandAndPredicate->getPredicate();
            commandAndPredicate->getPreHandler()->handle();
        }

        turnNCommand->run();

        if (turnNPredicate->test())
        {
            state = UFO_RUNNING_N_XDIVIDE2;
        }

        if (state != UFO_RUNNING_N_XDIVIDE2)
        {
            break;
        }
    }

    case UFO_RUNNING_N_XDIVIDE2: // Nから2/xまでの走行
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
            state = UFO_FINISHED;
        }

        if (state != UFO_FINISHED)
        {
            break;
        }
    }

    case UFO_FINISHED:
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
    return state == UFO_FINISHED;
}