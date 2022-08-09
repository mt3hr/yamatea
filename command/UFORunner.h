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
    DETECTING_OBSTACLE,
    CALCRATING,
    RUNNING_P_N,
    TURNNING_N,
    RUNNING_N_XDIVIDE2,
    FINISHED,
};

class UFORunner : public ObstacleDetectRunner, public FinishConfirmable
{
private:
    UFORunnerState state;
    WheelController *wheelController;
    SonarSensor *sonarSensor;

    Walker *p_nWalker;
    Walker *n_xdivide2Walker;
    Command *turnNCommand;

    DistancePredicate *p_nDistancePredicate;
    DistancePredicate *n_xdivide2DistancePreicate;
    Predicate *turnNPredicate;

    bool startedCalcrate = false;
    bool initedP_N = false;
    bool initedN_XDivide2 = false;
    bool initedTurnN = false;

    float p;
    float n;
    float pn;
    float ipn;

    int walkerPow;
    int rotatePow;

public:
    UFORunner(float p, float n, int walkerPow, int rotatePow, WheelController *wheelController, SonarSensor *sonarSensor, ObstacleDetector *obstacleDetector);
    ~UFORunner();
    void run() override;
    UFORunner *generateReverseCommand() override;
    bool isFinished() override;
};

#endif