#include "realtime_sipp.h"
#include "debug.h"

Realtime_SIPP::Realtime_SIPP(const Config& config)
    : AA_SIPP(config)
{}

SearchResult Realtime_SIPP::startSearch(Map& map, Task& task,
                                        DynamicObstacles& obstacles)
{
    focal_heuristic = Heuristic(config->connectedness);
    focal_heuristic.init(map.width, map.height, task.getNumberOfAgents());
    for (unsigned int numOfCurAgent = 0;
         numOfCurAgent < task.getNumberOfAgents(); numOfCurAgent++) {
        curagent = task.getAgent(numOfCurAgent);
        focal_heuristic.count(map, curagent);
    }
#ifdef __linux__
    timeval begin;
    timeval end;
    gettimeofday(&begin, nullptr);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    bool   solution_found(false);
    int    tries(0);
    int    bad_i(0);
    double timespent(0);
    priorities.clear();
    setPriorities(task);
    do {
        constraints = new Constraints(map.width, map.height);
        for (int k = 0; k < obstacles.getNumberOfObstacles(); k++) {
            constraints->addConstraints(obstacles.getSections(k),
                                        obstacles.getSize(k),
                                        obstacles.getMSpeed(k), map);
        }
        sresult.pathInfo.clear();
        sresult.pathInfo.resize(task.getNumberOfAgents());
        sresult.agents       = task.getNumberOfAgents();
        sresult.agentsSolved = 0;
        sresult.flowtime     = 0;
        sresult.makespan     = 0;
        for (int k = 0; k < task.getNumberOfAgents(); k++) {
            curagent = task.getAgent(k);
            constraints->setParams(curagent.size, curagent.mspeed,
                                   curagent.rspeed, config->planforturns,
                                   config->inflatecollisionintervals,
                                   config->planforturns);
            lineofsight.setSize(curagent.size);
            if (config->startsafeinterval > 0) {
                auto cells =
                  lineofsight.getCells(curagent.start_i, curagent.start_j);
                constraints->addStartConstraint(
                  curagent.start_i, curagent.start_j, config->startsafeinterval,
                  cells, curagent.size);
            }
        }
        for (unsigned int numOfCurAgent = 0;
             numOfCurAgent < task.getNumberOfAgents(); numOfCurAgent++) {
            curagent = task.getAgent(current_priorities[numOfCurAgent]);
            constraints->setParams(curagent.size, curagent.mspeed,
                                   curagent.rspeed, config->planforturns,
                                   config->inflatecollisionintervals,
                                   config->planforturns);
            lineofsight.setSize(curagent.size);
            if (config->startsafeinterval > 0) {
                auto cells =
                  lineofsight.getCells(curagent.start_i, curagent.start_j);
                constraints->removeStartConstraint(cells, curagent.start_i,
                                                   curagent.start_j);
            }
            if (findPath(current_priorities[numOfCurAgent], map)) {
                constraints->addConstraints(
                  sresult.pathInfo[current_priorities[numOfCurAgent]].sections,
                  curagent.size, curagent.mspeed, map);
            } else {
                bad_i = current_priorities[numOfCurAgent];
                break;
            }
            if (numOfCurAgent + 1 == task.getNumberOfAgents()) {
                solution_found = true;
            }
        }

        delete constraints;
        tries++;
#ifdef __linux__
        gettimeofday(&end, nullptr);
        timespent = (end.tv_sec - begin.tv_sec) +
                    static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        timespent = static_cast<double long>(end.QuadPart - begin.QuadPart) /
                    freq.QuadPart;
#endif
        if (timespent > config->timelimit) {
            break;
        }
    } while (changePriorities(bad_i) && !solution_found);

#ifdef __linux__
    gettimeofday(&end, nullptr);
    sresult.runtime =
      (end.tv_sec - begin.tv_sec) +
      static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.runtime =
      static_cast<double long>(end.QuadPart - begin.QuadPart) / freq.QuadPart;
#endif
    sresult.tries = tries;
    if (sresult.pathfound) {
        std::vector<conflict> confs = CheckConflicts(task);
        for (unsigned int i = 0; i < confs.size(); i++) {
            std::cout << confs[i].i << " " << confs[i].j << " " << confs[i].g
                      << " " << confs[i].agent1 << " " << confs[i].agent2
                      << "\n";
        }
    }
    return sresult;
}

bool Realtime_SIPP::findPath(unsigned int numOfCurAgent, const Map& map)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    close.clear();
    open.clear();
    constraints->use_likhachev = config->use_likhachev;
    ResultPathInfo resultPath;
    constraints->resetSafeIntervals(map.width, map.height);
    constraints->updateCellSafeIntervals({curagent.start_i, curagent.start_j});
    Node curNode(curagent.start_i, curagent.start_j, -1, 0, 0),
      goalNode(curagent.goal_i, curagent.goal_j, -1, CN_INFINITY, CN_INFINITY);
    curNode.F           = getHValue(curNode.i, curNode.j);
    curNode.interval    = constraints->getSafeInterval(curNode.i, curNode.j, 0);
    curNode.interval_id = curNode.interval.id;
    curNode.heading     = curagent.start_heading;
    curNode.optimal     = true;
    addOpen(curNode);
    int             reexpanded(0);
    int             close_id(0);
    std::list<Node> reexpanded_list;
    // real-time search loop
    int iterationCounter(0);
    DEBUG_MSG("lookahead limit " << config->fixedlookahead);
    while (iterationCounter++ < 10000) {
        // DEBUG_MSG_NO_LINE_BREAK( "iteration id " << iterationCounter);
        DEBUG_MSG("iteration id " << iterationCounter);
        DEBUG_MSG("search root i, j, g: " << curNode.i << " " << curNode.j
                                          << " " << curNode.g);

        // Tianyi note: this goal test might be too much simplified, check
        // AA_SIPP::stpCriterion
        if (curNode.i == curagent.goal_i && curNode.j == curagent.goal_j) {
            DEBUG_MSG("goal reached!");
            gettimeofday(&end, nullptr);
            resultPath.runtime =
              (end.tv_sec - begin.tv_sec) +
              static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;

            resultPath.sections        = hppath;
            resultPath.pathfound       = true;
            resultPath.expanded        = close.size();
            resultPath.generated       = open.size() + close.size();
            resultPath.path            = lppath;
            resultPath.iterationPath   = onlinePlanSections;
            resultPath.pathlength      = goalNode.g;
            resultPath.reopened        = constraints->reopened; //;
            resultPath.reexpanded      = reexpanded;
            resultPath.reexpanded_list = reexpanded_list;
            sresult.pathfound          = true;
            sresult.flowtime += goalNode.g;
            sresult.makespan = std::max(sresult.makespan, goalNode.g);
            sresult.pathInfo[numOfCurAgent] = resultPath;
            sresult.agentsSolved++;

            break;
        }

        timeval beginOfRealtimeCycle, endOfRealtimeCycle;
        gettimeofday(&beginOfRealtimeCycle, NULL);

        int curExpansion(0);
        // expansion phase
        while (!stopCriterion(curNode, goalNode) &&
               curExpansion < config->fixedlookahead) {
            curExpansion++;
            curNode = findMin();
            DEBUG_MSG("  current expansion "
                      << curExpansion << " expandNode i, j, g: " << curNode.i
                      << " " << curNode.j << " " << curNode.g);
            auto range = close.equal_range(curNode.i * map.width + curNode.j);
            for (auto it = range.first; it != range.second; it++) {
                if (it->second.interval_id ==
                    curNode.interval_id) // && it->second.heading_id ==
                                         // curNode.heading_id)
                {
                    reexpanded++;
                    reexpanded_list.push_back(curNode);
                    if (config->planforturns &&
                        it->second.heading_id != curNode.heading_id) {
                        reexpanded--;
                    }
                    break;
                }
            }
            curNode.close_id = close_id;
            close_id++;
            close.insert({curNode.i * map.width + curNode.j, curNode});
            for (Node s : findSuccessors(curNode, map)) {
                if (config->use_likhachev) {
                    range    = close.equal_range(s.i * map.width + s.j);
                    bool add = true;
                    for (auto it = range.first; it != range.second; it++) {
                        if (it->second.interval_id == curNode.interval_id &&
                            (!config->planforturns ||
                             it->second.heading_id == curNode.heading_id)) {
                            add = false;
                            break;
                        }
                    }
                    s.optimal = false;
                    if (add) {
                        addOpen(s);
                    }
                    if (curNode.optimal == true) {
                        s.optimal = true;
                        s.F = config->h_weight * (s.g + getHValue(s.i, s.j));
                        addOpen(s);
                    }
                } else {
                    DEBUG_MSG("    valid successor node " << s.i << " " << s.j
                                                          << " " << s.g);
                    addOpen(s);
                    DEBUG_MSG("    open size " << open.size());
                }
            }
        }

        gettimeofday(&endOfRealtimeCycle, nullptr);

        // decision-making phase
        // 1) commit the the toplevel action that would lead to the best search
        // frontier node
        //
        auto bestTLA = backupAndRecordPartialPlan(curNode, beginOfRealtimeCycle,
                                                  endOfRealtimeCycle);

        // 2) re-root the search tree to the best successor node
        curNode = bestTLA;

        curNode.Parent = nullptr;
        open.clear();
        addOpen(curNode);
        DEBUG_MSG("open size " << open.size());
        curExpansion = 0;
        Node resetGoalNode(curagent.goal_i, curagent.goal_j, -1, CN_INFINITY,
                           CN_INFINITY);
        goalNode = resetGoalNode;

        // learning phase
        // update the heuristic in closed list
        close.clear();
        close_id = 0;
    }
    if (!resultPath.pathfound) {
        gettimeofday(&end, nullptr);
        resultPath.runtime =
          (end.tv_sec - begin.tv_sec) +
          static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
        std::cout << "Path for agent " << curagent.id << " not found!\n";
        sresult.pathfound    = false;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength           = 0;
        sresult.pathInfo[numOfCurAgent] = resultPath;
    }

    return resultPath.pathfound;
}

void Realtime_SIPP::recordToPrimaryPath(Node committedNode)
{
    auto curNode = committedNode;

    std::list<Node> path;
    path.push_front(curNode);
    if (curNode.Parent != nullptr) {
        curNode = *curNode.Parent;
        if (curNode.Parent != nullptr) {
            do {
                path.push_front(curNode);
                curNode = *curNode.Parent;
            } while (curNode.Parent != nullptr);
        }
        path.push_front(curNode);
    }
    for (auto it = path.begin(); it != path.end(); it++) {
        hppath.push_back(*it);
    }
    if (config->planforturns && curagent.goal_heading >= 0) {
        Node add    = hppath.back();
        add.heading = curagent.goal_heading;
        hppath.back().g -=
          getRCost(hppath.back().heading, curagent.goal_heading);
        hppath.push_back(add);
    }
    for (unsigned int i = 1; i < hppath.size(); i++) {
        if ((hppath[i].g -
             (hppath[i - 1].g + getCost(hppath[i].i, hppath[i].j,
                                        hppath[i - 1].i, hppath[i - 1].j) /
                                  curagent.mspeed)) > CN_EPSILON) {
            Node add   = hppath[i - 1];
            add.Parent = hppath[i].Parent;
            add.g      = hppath[i].g - getCost(hppath[i].i, hppath[i].j,
                                          hppath[i - 1].i, hppath[i - 1].j) /
                                    curagent.mspeed;
            add.heading = hppath[i].heading;
            hppath.emplace(hppath.begin() + i, add);
            i++;
        }
    }
    if (config->planforturns && curagent.goal_heading >= 0) {
        hppath.pop_back();
    }
}

void Realtime_SIPP::recordToSecondaryPath(Node committedNode)
{
    auto curNode = committedNode;
    if (curNode.Parent != nullptr) {
        std::vector<Node> lineSegment;
        do {
            calculateLineSegment(lineSegment, *curNode.Parent, curNode);
            lppath.insert(lppath.begin(), ++lineSegment.begin(),
                          lineSegment.end());
            curNode = *curNode.Parent;
        } while (curNode.Parent != nullptr);
        lppath.push_front(*lineSegment.begin());
    } else {
        lppath.push_front(curNode);
    }
}

void Realtime_SIPP::recordToOnlinePath(Node frontierNode, const timeval& begin,
                                       const timeval& end)
{
    auto curNode = frontierNode;

    std::list<Node>   path;
    std::vector<Node> sections;
    path.push_front(curNode);
    if (curNode.Parent != nullptr) {
        curNode = *curNode.Parent;
        if (curNode.Parent != nullptr) {
            do {
                path.push_front(curNode);
                curNode = *curNode.Parent;
            } while (curNode.Parent != nullptr);
        }
        path.push_front(curNode);
    }
    for (auto it = path.begin(); it != path.end(); it++) {
        sections.push_back(*it);
    }
    if (config->planforturns && curagent.goal_heading >= 0) {
        Node add    = sections.back();
        add.heading = curagent.goal_heading;
        sections.back().g -=
          getRCost(sections.back().heading, curagent.goal_heading);
        sections.push_back(add);
    }
    for (unsigned int i = 1; i < sections.size(); i++) {
        if ((sections[i].g - (sections[i - 1].g +
                              getCost(sections[i].i, sections[i].j,
                                      sections[i - 1].i, sections[i - 1].j) /
                                curagent.mspeed)) > CN_EPSILON) {
            Node add   = sections[i - 1];
            add.Parent = sections[i].Parent;
            add.g =
              sections[i].g - getCost(sections[i].i, sections[i].j,
                                      sections[i - 1].i, sections[i - 1].j) /
                                curagent.mspeed;
            add.heading = sections[i].heading;
            sections.emplace(sections.begin() + i, add);
            i++;
        }
    }
    if (config->planforturns && curagent.goal_heading >= 0) {
        sections.pop_back();
    }

    ResultPathInfo partialPath;
    partialPath.runtime =
      (end.tv_sec - begin.tv_sec) +
      static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;

    partialPath.sections  = sections;
    partialPath.pathfound = true;
    partialPath.expanded  = close.size(); // accumulated closed
    partialPath.generated =
      open.size() +
      close.size(); // this is wrong becauwse open is cleaned up every iteration
    partialPath.path       = path;
    partialPath.pathlength = frontierNode.g;
    onlinePlanSections.push_back(partialPath);
}

Node Realtime_SIPP::backupAndRecordPartialPlan(const Node&    curNode,
                                               const timeval& begin,
                                               const timeval& end)
{
    // Tianyi note: this goal test might be too much simplified, check
    // AA_SIPP::stpCriterion
    auto bestFrontierNode = findMin();

    DEBUG_MSG("goal i, j: " << curagent.goal_i << " " << curagent.goal_j);

    if (curNode.i == curagent.goal_i && curNode.j == curagent.goal_j) {
        bestFrontierNode = curNode;
    }

    auto cur       = bestFrontierNode;
    auto parentPtr = bestFrontierNode.Parent;

    while (parentPtr != nullptr && parentPtr->Parent != nullptr) {
        cur       = *parentPtr;
        parentPtr = cur.Parent;
    }

    recordToOnlinePath(bestFrontierNode, begin, end);
    recordToPrimaryPath(cur);
    recordToSecondaryPath(cur);

    DEBUG_MSG("curNode after search i, j, g: " << curNode.i << " " << curNode.j
                                               << " " << curNode.g);
    DEBUG_MSG("best frontier i, j, g: " << bestFrontierNode.i << " "
                                        << bestFrontierNode.j << " "
                                        << bestFrontierNode.g);
    DEBUG_MSG("best TLA after search i, j, g: " << cur.i << " " << cur.j << " "
                                                << cur.g);

    return cur;
}
