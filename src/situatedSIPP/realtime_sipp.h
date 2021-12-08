#pragma once
#include "../aa_sipp.h"
#include "decisionAlgorithms/miniminbackup.hpp"
#include "learningAlgorithms/DijkstraLearning.hpp"
#include "learningAlgorithms/PlrtaLearning.hpp"
#include "learningAlgorithms/noLearning.hpp"
#include "structs.h"
#include <boost/functional/hash.hpp>

SearchResult rtsr2sr(const RTSearchResult& rtsr);

class Realtime_SIPP : public AA_SIPP
{

public:
    Realtime_SIPP(const Config& config);
    SearchResult startSearch(Map& map, Task& task, DynamicObstacles& obstacles);
    RTSearchResult startRTSearch(Map& map, Task& task,
                                 DynamicObstacles& obstacles);
    bool   findPath(unsigned int numOfCurAgent, const Map& map) override;
    RTNode findMin();
    bool   stopCriterion(const RTNode& curNode, RTNode& goalNode);
    double calcHeading(const RTNode& node, const RTNode& son);
    void recordToOnlinePath(const RTNode& rootNode, const RTNode& frontierNode,
                            const timeval& begin, const timeval& end);

    std::list<RTNode> findSuccessors(const RTNode curNode, const Map& map);
    virtual void      makePrimaryPath(RTNode curNode);
    virtual void      makeSecondaryPath(RTNode curNode);
    void   calculateLineSegment(std::vector<RTNode>& line, const RTNode& start,
                                const RTNode& goal);
    RTNode resetParent(RTNode current, RTNode Parent, const Map& map);
    std::vector<conflict> CheckConflicts(
      const Task& task); // bruteforce checker. It splits final(already built)
                         // trajectories into sequences of points and checks
                         // distances between them
    void                                 update_focal(double cost);
    std::unordered_multimap<int, RTNode> close;
    std::list<RTNode>                    lppath;
    std::vector<RTNode>                  hppath;
    RTSearchResult                       sresult;

private:
    void                               addOpen(RTNode& newNode);
    RTOPEN_container                   open;
    std::vector<RTResultPathInfo>      onlinePlanSections;
    std::shared_ptr<LearningAlgorithm> learningModulePtr;
    // RTConstraints *constraints;
    std::unordered_map<std::string, std::shared_ptr<LearningAlgorithm>>
      map_configStringToLearningModule{
        {"nolearning", std::make_shared<NoLearning>()},
        {"dijkstralearning", std::make_shared<DijkstraLearning>()},
        {"plrtalearning", std::make_shared<PlrtaLearning>()}};

    std::shared_ptr<DecisionAlgorithm<Realtime_SIPP>> decisionModulePtr;
    std::unordered_map<std::string,
                       std::shared_ptr<DecisionAlgorithm<Realtime_SIPP>>>
      map_configStringToDecisionModule{
        {"miniminbackup", std::make_shared<MiniminBackup<Realtime_SIPP>>()}};
};
