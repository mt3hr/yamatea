#include "DistanceReader.h"
#include "SonarSensor.h"
#include "PrintMessage.h"
#include "sstream"

using namespace ev3api;
using namespace std;

DistanceReader::DistanceReader(SonarSensor *ss)
{
    sonarSensor = ss;
}

void DistanceReader::run()
{
    distanceValue = sonarSensor->getDistance();

    // 出力処理。変数名は雑。
    stringstream ds;
    ds << "distance:" << float(distanceValue); // intのままだと出力されないのでfloatに変換する

    vector<string> messageLines;
    messageLines.push_back("distance reader");
    messageLines.push_back(ds.str());

    PrintMessage printMessage(messageLines, true);
    printMessage.run();
}

DistanceReader *DistanceReader::generateReverseCommand()
{
    return new DistanceReader(sonarSensor);
}
