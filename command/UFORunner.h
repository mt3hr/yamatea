#ifndef UFORunner_H
#define UFORunner_H

#include "ObstacleDetectRunner.h"
#include "ObstacleDetector.h"
#include "WheelController.h"
#include "SonarSensor.h"
#include "FinishConfirmable.h"
#include "DistancePredicate.h"
#include "Predicate.h"
#include "Walker.h"
#include "Command.h"

enum UFORunnerState
{
    UFO_DETECTING_OBSTACLE,
    UFO_CALCRATING,
    UFO_TURNNIN_TO_P,
    UFO_TURNNING_P_IPN,
    UFO_RUNNING_P_N,
    UFO_TURNNING_N,
    UFO_RUNNING_N_XDIVIDE2,
    UFO_FINISHED,
};

float toRadian(float degree);
float toDegree(float radian);

class UFORunner : public ObstacleDetectRunner, public FinishConfirmable
{
private:
    UFORunnerState state;
    WheelController *wheelController;
    SonarSensor *sonarSensor;

    Command *turnToPCommand;
    Command *turnPIPNCommand;
    Walker *p_nWalker;
    Walker *n_xdivide2Walker;
    Command *turnNCommand;

    Predicate *turnToPPredicate;
    Predicate *turnPIPNPredicate;
    DistancePredicate *p_nDistancePredicate;
    DistancePredicate *n_xdivide2DistancePreicate;
    Predicate *turnNPredicate;

    bool initedTurnToP = false;
    bool initedTurnPIPN = false;
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
    float pin;
    float nTurnAngleACosArg; // TODO けして
    float iArg;              // TODO 消して
    float dinArg;            // TODO 消して
    float nTurnAngle;

    int walkerPow;
    int rotatePow;

public:
    UFORunner(float n, int walkerPow, int rotatePow, WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    ~UFORunner();
    void run() override;
    UFORunner *generateReverseCommand() override;
    bool isFinished() override;
};

#endif