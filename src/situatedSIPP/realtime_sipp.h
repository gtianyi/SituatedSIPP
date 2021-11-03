#pragma once
#include "../aa_sipp.h"
#include "../structs.h"
#include "learningAlgorithms/DijkstraLearning.hpp"
#include "learningAlgorithms/noLearning.hpp"
#include <boost/functional/hash.hpp>

class SIPPState{
public:
  int i{}, j{};
  SafeInterval interval;
  SIPPState(const Node& n){
    i = n.i;
    j = n.j;
    interval = n.interval;
  }
  auto operator==(const SIPPState &other) -> bool;
  auto hash_value(SIPPState const& n) -> std::size_t;
  };

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
      map_configStringTolearningModule{
        {"nolearning", std::make_shared<NoLearning>()},
        {"dijkstralearning", std::make_shared<DijkstraLearning>()}};
};
