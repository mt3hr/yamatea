#include "RawColorPredicate.h"
#include "Sensor.h"
#include "RobotAPI.h"
#include "Setting.h"
#include "DebugUtil.h"

using namespace ev3api;

RawColorPredicate::RawColorPredicate(int *r, RawColorPredicateCondition rCondition, int *g, RawColorPredicateCondition gCondition, int *b, RawColorPredicateCondition bCondition)
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
        rOK = *r <= rawColor.r;
        break;
    }
    case LESS_THAN:
    {
        rOK = *r >= rawColor.r;
        break;
    }
    case BETWEEN3:
    {
        rOK = *r + 3 >= rawColor.r && *r - 3 <= rawColor.r;
        break;
    }
    case BETWEEN5:
    {
        rOK = *r + 5 >= rawColor.r && *r - 5 <= rawColor.r;
        break;
    }
    case BETWEEN10:
    {
        rOK = *r + 10 >= rawColor.r && *r - 10 <= rawColor.r;
        break;
    }
    case BETWEEN15:
    {
        rOK = *r + 15 >= rawColor.r && *r - 15 <= rawColor.r;
        break;
    }
    case BETWEEN20:
    {
        rOK = *r + 20 >= rawColor.r && *r - 20 <= rawColor.r;
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
        gOK = *g <= rawColor.g;
        break;
    }
    case LESS_THAN:
    {
        gOK = *g >= rawColor.g;
        break;
    }
    case BETWEEN3:
    {
        gOK = *g + 3 >= rawColor.g && *g - 3 <= rawColor.g;
        break;
    }
    case BETWEEN5:
    {
        gOK = *g + 5 >= rawColor.g && *g - 5 <= rawColor.g;
        break;
    }
    case BETWEEN10:
    {
        gOK = *g + 10 >= rawColor.g && *g - 10 <= rawColor.g;
        break;
    }
    case BETWEEN15:
    {
        gOK = *g + 15 >= rawColor.g && *g - 15 <= rawColor.g;
        break;
    }
    case BETWEEN20:
    {
        gOK = *g + 20 >= rawColor.g && *g - 20 <= rawColor.g;
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
        bOK = *b <= rawColor.b;
        break;
    }
    case LESS_THAN:
    {
        bOK = *b >= rawColor.b;
        break;
    }
    case BETWEEN3:
    {
        bOK = *b + 3 >= rawColor.b && *b - 3 <= rawColor.b;
        break;
    }
    case BETWEEN5:
    {
        bOK = *b + 5 >= rawColor.b && *b - 5 <= rawColor.b;
        break;
    }
    case BETWEEN10:
    {
        bOK = *b + 10 >= rawColor.b && *b - 10 <= rawColor.b;
        break;
    }
    case BETWEEN15:
    {
        bOK = *b + 15 >= rawColor.b && *b - 15 <= rawColor.b;
        break;
    }
    case BETWEEN20:
    {
        bOK = *b + 20 >= rawColor.b && *b - 20 <= rawColor.b;
        break;
    }
    }

    writeDebug("RawColorPredicate");
    writeEndLineDebug();
    writeDebug("r: ");
    writeDebug(rawColor.r);
    writeEndLineDebug();
    writeDebug("g: ");
    writeDebug(rawColor.g);
    writeEndLineDebug();
    writeDebug("b: ");
    writeDebug(rawColor.b);
    writeEndLineDebug();
    flushDebug(TRACE, robotAPI);

    return rOK && gOK && bOK;
}

void RawColorPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

RawColorPredicate *RawColorPredicate::generateReversePredicate()
{
    return new RawColorPredicate(r, rCondition, g, gCondition, b, bCondition);
}

RedPredicate::RedPredicate() : RawColorPredicate(&redR, redRCondition, &redG, redGCondition, &redB, redBCondition){};
GreenPredicate::GreenPredicate() : RawColorPredicate(&greenR, greenRCondition, &greenG, greenGCondition, &greenB, greenBCondition){};
BluePredicate::BluePredicate() : RawColorPredicate(&blueR, blueRCondition, &blueG, blueGCondition, &blueB, blueBCondition){};
YellowPredicate::YellowPredicate() : RawColorPredicate(&yellowR, yellowRCondition, &yellowG, yellowGCondition, &yellowB, yellowBCondition){};
BlueEdgePredicate::BlueEdgePredicate() : RawColorPredicate(&blueWhiteEdgeR, blueWhiteEdgeRCondition, &blueWhiteEdgeG, blueWhiteEdgeGCondition, &blueWhiteEdgeB, blueWhiteEdgeBCondition){};
BlackPredicate::BlackPredicate() : RawColorPredicate(&blackR, blackRCondition, &blackG, blackGCondition, &blackB, blackBCondition){};
WhiteAtSlaromPredicate::WhiteAtSlaromPredicate() : RawColorPredicate(&whiteAtSlalomR, whiteAtSlalomRCondition, &whiteAtSlalomG, whiteAtSlalomGCondition, &whiteAtSlalomB, whiteAtSlalomBCondition){};
GrayPredicate::GrayPredicate() : RawColorPredicate(&grayR, grayRCondition, &grayG, grayGCondition, &grayB, grayBCondition){};

RedPredicate::~RedPredicate(){};
GreenPredicate::~GreenPredicate(){};
BluePredicate::~BluePredicate(){};
YellowPredicate::~YellowPredicate(){};
BlueEdgePredicate::~BlueEdgePredicate(){};
BlackPredicate::~BlackPredicate(){};
WhiteAtSlaromPredicate::~WhiteAtSlaromPredicate(){};
GrayPredicate::~GrayPredicate(){};

RedCardPredicate::RedCardPredicate() : RawColorPredicate(&redCardR, redCardRCondition, &redCardG, redCardGCondition, &redCardB, redCardBCondition){};
GreenCardPredicate::GreenCardPredicate() : RawColorPredicate(&greenCardR, greenCardRCondition, &greenCardG, greenCardGCondition, &greenCardB, greenCardBCondition){};
BlueCardPredicate::BlueCardPredicate() : RawColorPredicate(&blueCardR, blueCardRCondition, &blueCardG, blueCardGCondition, &blueCardB, blueCardBCondition){};
YellowCardPredicate::YellowCardPredicate() : RawColorPredicate(&yellowCardR, yellowCardRCondition, &yellowCardG, yellowCardGCondition, &yellowCardB, yellowCardBCondition){};
RedCardPredicate::~RedCardPredicate(){};
GreenCardPredicate::~GreenCardPredicate(){};
BlueCardPredicate::~BlueCardPredicate(){};
YellowCardPredicate::~YellowCardPredicate(){};

