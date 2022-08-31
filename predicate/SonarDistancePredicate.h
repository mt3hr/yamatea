#ifndef SonarDistancePredicate_H
#define SonarDistancePredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

class SonarDistancePredicate : public Predicate
{
private:
    int distance;
    bool lessThan;

public:
    SonarDistancePredicate(int distance, bool lessThan);
    virtual ~SonarDistancePredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual Predicate *generateReversePredicate();
};

#endif