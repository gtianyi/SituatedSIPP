#include "realtime_sipp.h"

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
    while (!stopCriterion(curNode, goalNode)) {
        curNode    = findMin();
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
                    s.F       = config->h_weight * (s.g + getHValue(s.i, s.j));
                    addOpen(s);
                }
            } else {
                addOpen(s);
            }
        }
    }
    if (goalNode.g < CN_INFINITY) {
        makePrimaryPath(goalNode);
#ifdef __linux__
        gettimeofday(&end, nullptr);
        resultPath.runtime =
          (end.tv_sec - begin.tv_sec) +
          static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime =
          static_cast<double long>(end.QuadPart - begin.QuadPart) /
          freq.QuadPart;
#endif
        resultPath.sections = hppath;
        makeSecondaryPath(goalNode);
        resultPath.pathfound       = true;
        resultPath.expanded        = close.size();
        resultPath.generated       = open.size() + close.size();
        resultPath.path            = lppath;
        resultPath.pathlength      = goalNode.g;
        resultPath.reopened        = constraints->reopened; //;
        resultPath.reexpanded      = reexpanded;
        resultPath.reexpanded_list = reexpanded_list;
        sresult.pathfound          = true;
        sresult.flowtime += goalNode.g;
        sresult.makespan = std::max(sresult.makespan, goalNode.g);
        sresult.pathInfo[numOfCurAgent] = resultPath;
        sresult.agentsSolved++;
    } else {
#ifdef __linux__
        gettimeofday(&end, nullptr);
        resultPath.runtime =
          (end.tv_sec - begin.tv_sec) +
          static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.runtime =
          static_cast<double long>(end.QuadPart - begin.QuadPart) /
          freq.QuadPart;
#endif
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
