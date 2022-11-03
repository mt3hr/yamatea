#include "CardColorReaderUseRaw.h"
#include "Command.h"
#include "RobotAPI.h"
#include "RawColorPredicate.h"
#include "DebugUtil.h"
#include "ColorIDReader.h"

CardColorReaderUseRaw::CardColorReaderUseRaw()
{
    color = new colorid_t(COLOR_RED);
};

CardColorReaderUseRaw::~CardColorReaderUseRaw()
{
    delete redCardPredicate;
    delete greenCardPredicate;
    delete blueCardPredicate;
    delete yellowCardPredicate;
    delete color;
};

void CardColorReaderUseRaw::run(RobotAPI *robotAPI)
{
    if (redCardPredicate->test(robotAPI))
    {
        *color = COLOR_RED;
    }
    else if (greenCardPredicate->test(robotAPI))
    {
        *color = COLOR_GREEN;
    }
    else if (blueCardPredicate->test(robotAPI))
    {
        *color = COLOR_BLUE;
    }
    else if (yellowCardPredicate->test(robotAPI))
    {
        *color = COLOR_YELLOW;
    }
    writeDebug("ColorReader: ");
    writeEndLineDebug();
    writeDebug("color id use raw: ");
    writeDebug(((int)(*color)));
    writeEndLineDebug();
    writeDebug("color: ");
    writeDebug(colorIDToString(*color));
    flushDebug(INFO, robotAPI);
    return;
}

void CardColorReaderUseRaw::preparation(RobotAPI *robotAPI)
{
    return;
}

CardColorReaderUseRaw *CardColorReaderUseRaw::generateReverseCommand()
{
    return new CardColorReaderUseRaw();
}

colorid_t *CardColorReaderUseRaw::getColorPtr()
{
    return color;
}
