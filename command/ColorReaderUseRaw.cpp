#include "ColorReaderUseRaw.h"
#include "Command.h"
#include "RobotAPI.h"
#include "RawColorPredicate.h"
#include "DebugUtil.h"
#include "ColorIDReader.h"

ColorReaderUseRaw::ColorReaderUseRaw()
{
    color = new colorid_t(COLOR_RED);
};

ColorReaderUseRaw::~ColorReaderUseRaw()
{
    delete redPredicate;
    delete greenPredicate;
    delete bluePredicate;
    delete yellowPredicate;
    delete color;
};

void ColorReaderUseRaw::run(RobotAPI *robotAPI)
{
    if (redPredicate->test(robotAPI))
    {
        *color = COLOR_RED;
    }
    else if (greenPredicate->test(robotAPI))
    {
        *color = COLOR_GREEN;
    }
    else if (bluePredicate->test(robotAPI))
    {
        *color = COLOR_BLUE;
    }
    else if (yellowPredicate->test(robotAPI))
    {
        *color = COLOR_YELLOW;
    }
    writeDebug("ColorReader: ");
    writeEndLineDebug();
    writeDebug("color id use raw: ");
    writeDebug(((int)(*color)));
    writeDebug("color: ");
    writeDebug(colorIDToString(*color));
    flushDebug(DEBUG, robotAPI);
    return;
}

void ColorReaderUseRaw::preparation(RobotAPI *robotAPI)
{
    return;
}

ColorReaderUseRaw *ColorReaderUseRaw::generateReverseCommand()
{
    return new ColorReaderUseRaw();
}

colorid_t *ColorReaderUseRaw::getColorPtr()
{
    return color;
}
