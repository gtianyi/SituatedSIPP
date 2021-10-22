#pragma once
#include "../aa_sipp.h"
#include "learningAlgorithms/DijkstraLearning.hpp"
#include "learningAlgorithms/noLearning.hpp"

class Realtime_SIPP : public AA_SIPP
{

public:
    Realtime_SIPP(const Config& config);

    SearchResult startSearch(Map& map, Task& task,
                             DynamicObstacles& obstacles) override;
    bool         findPath(unsigned int numOfCurAgent, const Map& map) override;

private:
    Node backupAndRecordPartialPlan(const Node& curNode, const timeval& begin,
                                    const timeval& end);
    void recordToOnlinePath(const Node& rootNode, const Node& frontierNode,
                            const timeval& begin, const timeval& end);

    std::vector<ResultPathInfo>        onlinePlanSections;
    std::shared_ptr<LearningAlgorithm> learningModulePtr;
    std::unordered_map<std::string, std::shared_ptr<LearningAlgorithm>>
      configStringTolearningModule{
        {"nolearning", std::make_shared<NoLearning>()},
        {"dijkstralearning", std::make_shared<DijkstraLearning>()}};
};
