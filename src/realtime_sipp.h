#pragma once
#include "aa_sipp.h"

class Realtime_SIPP : public AA_SIPP
{

public:
    Realtime_SIPP(const Config& config);

    SearchResult startSearch(Map& map, Task& task,
                             DynamicObstacles& obstacles) override;
    bool         findPath(unsigned int numOfCurAgent, const Map& map) override;

private:
    Node backupAndRecordPartialPlan(const Node& curNode, const timeval& begin, const timeval& end);
    void recordToOnlinePath(const Node& rootNode, const Node& frontierNode, const timeval& begin,
                            const timeval& end);

    std::vector<ResultPathInfo> onlinePlanSections;
};
