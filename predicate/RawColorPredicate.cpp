#include "RawColorPredicate.h"
#include "Sensor.h"
#include "RobotAPI.h"
#include "Setting.h"

using namespace ev3api;

RawColorPredicate::RawColorPredicate(int r, RawColorPredicateCondition rCondition, int g, RawColorPredicateCondition gCondition, int b, RawColorPredicateCondition bCondition)
{
    this->r = r;
    this->rCondition = rCondition;
    this->g = g;
    this->gCondition = gCondition;
    this->b = b;
    this->bCondition = bCondition;
};

RawColorPredicate::~RawColorPredicate(){

};

bool RawColorPredicate::test(RobotAPI *robotAPI)
{
    bool rOK = false;
    bool gOK = false;
    bool bOK = false;

    rgb_raw_t rawColor;
    robotAPI->getColorSensor()->getRawColor(rawColor);

    switch (rCondition)
    {
    case IGNORE:
    {
        rOK = true;
        break;
    }
    case GREATER_THAN:
    {
        rOK = r <= rawColor.r;
        break;
    }
    case LESS_THAN:
    {
        rOK = r >= rawColor.r;
        break;
    }
    case BETWEEN5:
    {
        rOK = r + 5 >= rawColor.r && r - 5 >= rawColor.r;
        break;
    }
    case BETWEEN10:
    {
        rOK = r + 10 >= rawColor.r && r - 10 >= rawColor.r;
        break;
    }
    }

    switch (gCondition)
    {
    case IGNORE:
    {
        gOK = true;
        break;
    }
    case GREATER_THAN:
    {
        gOK = g <= rawColor.g;
        break;
    }
    case LESS_THAN:
    {
        gOK = g >= rawColor.g;
        break;
    }
    case BETWEEN5:
    {
        gOK = g + 5 >= rawColor.g && g - 5 >= rawColor.g;
        break;
    }
    case BETWEEN10:
    {
        gOK = g + 10 >= rawColor.g && g - 10 >= rawColor.g;
        break;
    }
    }

    switch (bCondition)
    {
    case IGNORE:
    {
        bOK = true;
        break;
    }
    case GREATER_THAN:
    {
        bOK = b <= rawColor.b;
        break;
    }
    case LESS_THAN:
    {
        bOK = b >= rawColor.b;
        break;
    }
    case BETWEEN5:
    {

        bOK = b + 5 >= rawColor.b && b - 5 >= rawColor.b;
        break;
    }
    case BETWEEN10:
    {
        bOK = b + 10 >= rawColor.b && b - 10 >= rawColor.b;
        break;
    }
    }

    return rOK && gOK && bOK;
}

void RawColorPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

Predicate *RawColorPredicate::generateReversePredicate()
{
    return new RawColorPredicate(r, rCondition, g, gCondition, b, bCondition);
}

BluePredicate::BluePredicate() : RawColorPredicate(b_r, b_rCondition, b_g, b_gCondition, b_b, b_bCondition){};
RedPredicate::RedPredicate() : RawColorPredicate(r_r, r_rCondition, r_g, r_gCondition, r_b, r_bCondition){};
GreenPredicate::GreenPredicate() : RawColorPredicate(b_r, b_rCondition, b_g, b_gCondition, b_b, b_bCondition){};
YellowPredicate::YellowPredicate() : RawColorPredicate(y_r, y_rCondition, y_g, y_gCondition, y_b, y_bCondition){};

BluePredicate::~BluePredicate(){};
RedPredicate::~RedPredicate(){};
GreenPredicate::~GreenPredicate(){};
YellowPredicate::~YellowPredicate(){};