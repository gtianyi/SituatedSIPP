#pragma once
#include "aa_sipp.h"

class Realtime_SIPP : public AA_SIPP
{

public:

    Realtime_SIPP(const Config &config);
    SearchResult startSearch(Map& map, Task& task, DynamicObstacles& obstacles) override;
    bool findPath(unsigned int numOfCurAgent, const Map& map) override;
};
