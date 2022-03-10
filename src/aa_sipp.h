#pragma once
#ifndef AA_SIPP_H
#define AA_SIPP_H

#include <cmath>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include <random>
#include "constraints.h"
#include "lineofsight.h"
#include "config.h"
#include "searchresult.h"
#include "task.h"
#include "dynamicobstacles.h"
#include "heuristic.h"
#ifdef __linux__
    #include <sys/time.h>
#else
    #include <windows.h>
#endif

void debug_open(const OPEN_container& open);

class AA_SIPP
{

public:

    AA_SIPP(const Config &config);
    virtual ~AA_SIPP();
    virtual SearchResult startSearch(Map &map, Task &task, DynamicObstacles &obstacles, SafeIntervals & safe_intervals);
    SearchResult sresult;

    template <typename T>
    friend class ExpansionAlgorithm;
    std::shared_ptr<const Config> config;
    double getHValue(int i, int j);

protected:
    void addOpen(Node &newNode);
    Node findMin();
    bool stopCriterion(const Node &curNode, Node &goalNode);
    bool testGoal(const Node &curNode, Node &goalNode);  // this is never defined?
    double getCost(int a_i, int a_j, int b_i, int b_j);
    double getRCost(double headingA, double headingB);
    double calcHeading(const Node &node, const Node &son);
    std::list<Node> findSuccessors(const Node curNode, const Map &map);
    virtual void makePrimaryPath(Node curNode);
    virtual void makeSecondaryPath(Node curNode);
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);
    void addConstraints(){}
    Node resetParent(Node current, Node Parent, const Map &map);
    virtual bool findPath(unsigned int numOfCurAgent, const Map &map, SafeIntervals & safe_intervals);
    std::vector<conflict> CheckConflicts(const Task &task);//bruteforce checker. It splits final(already built) trajectories into sequences of points and checks distances between them
    void setPriorities(const Task &task);
    bool changePriorities(int bad_i);
    void update_focal(double cost);
    Heuristic focal_heuristic;
    Focal_container focal;
    OPEN_container open;
    int open_id;
    std::unordered_multimap<int, Node> close;
    std::list<Node> lppath;
    std::vector<Node> hppath;
    std::vector<std::vector<int>> priorities;
    std::vector<int> current_priorities;
    LineOfSight lineofsight;
    Agent curagent;
    Constraints *constraints;
};

#endif // AA_SIPP_H
