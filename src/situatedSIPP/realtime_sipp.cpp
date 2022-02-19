#include "realtime_sipp.h"
#include "../debug.h"
#include "structs.h"
#include <math.h> 

#include <cstddef>
#include <string>
#include <fstream>

Realtime_SIPP::Realtime_SIPP(const Config& config_)
    : AA_SIPP(config_)
{
    constraints = nullptr;
    if (map_configStringToLearningModule.find(config->learningalgorithm) ==
        map_configStringToLearningModule.end()) {
        std::cerr << "unknown learning algorithm!";
        exit(0);
    }
    learningModulePtr =
      map_configStringToLearningModule[config->learningalgorithm];
    if (map_configStringToDecisionModule.find(config->decisionalgorithm) ==
        map_configStringToDecisionModule.end()) {
        std::cerr << "unknown decision algorithm!";
        exit(0);
    }
    decisionModulePtr =
      map_configStringToDecisionModule[config->decisionalgorithm];
    RTNode::set_dynmode(config->dynmode);
    if (map_configStringToExpansionModule.find(config->expansionalgorithm) ==
        map_configStringToExpansionModule.end()) {
        std::cerr << "unknown expansion algorithm!";
        exit(0);
    }
    expansionModulePtr =
      map_configStringToExpansionModule[config->expansionalgorithm];
    RTNode::set_expansion_order(config->expansionalgorithm);
    RTNode::set_dynmode(config->dynmode);
    lookaheadBudget = config->fixedlookahead;
}

SearchResult rtsr2sr(const RTSearchResult& rtsr)
{
    // convert the realtime search results to a search result.
    SearchResult sr;
    sr.pathfound    = rtsr.pathfound;
    sr.agentFate    = rtsr.agentFate;
    sr.makespan     = rtsr.makespan;
    sr.flowtime     = rtsr.flowtime;
    sr.runtime      = rtsr.runtime;
    sr.agents       = rtsr.agents;
    sr.agentsSolved = rtsr.agentsSolved;
    sr.tries        = rtsr.tries;
    sr.expansions = rtsr.expansions;
    sr.steps = rtsr.steps;
    sr.timingInformation = rtsr.timingInformation;
    sr.pathInfo.clear();
    for (auto path_info : rtsr.pathInfo) {
        sr.pathInfo.push_back(path_info.toRPI());
    }
    return sr;
}

SearchResult Realtime_SIPP::startSearch(Map& map, Task& task,
                                        DynamicObstacles& obstacles)
{
    RTSearchResult rtsr = startRTSearch(map, task, obstacles);
    return rtsr2sr(rtsr);
    // return AA_SIPP::sresult;  // this is probably not great to do.  //turns
    // out it was not
}

RTSearchResult Realtime_SIPP::startRTSearch(Map& map, Task& task,
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
    learningModulePtr->setAgentSpeed(curagent.mspeed);
    do {
        timer.resume_si();
        constraints = new Constraints(map.width, map.height);
        for (int k = 0; k < obstacles.getNumberOfObstacles(); k++) {
            constraints->addConstraints(obstacles.getSections(k),
                                        obstacles.getSize(k),
                                        obstacles.getMSpeed(k), map);
        }
        timer.stop_si();
        sresult.pathInfo.clear();
        sresult.pathInfo.resize(task.getNumberOfAgents());
        sresult.agents       = task.getNumberOfAgents();
        sresult.agentsSolved = 0;
        sresult.flowtime     = 0;
        sresult.makespan     = 0;
        timer.resume_si();
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
        timer.stop_si();
        timer.resume_si();
        for (unsigned int numOfCurAgent = 0; numOfCurAgent < task.getNumberOfAgents(); numOfCurAgent++) {
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
            timer.stop_si();
            if (findPath(current_priorities[numOfCurAgent], map)) {
                timer.resume_si();
                constraints->addConstraints(
                  sresult.pathInfo[current_priorities[numOfCurAgent]].sections,
                  curagent.size, curagent.mspeed, map);
                timer.stop_si();
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
     DEBUG_MSG_RED(sresult.agentFate);
    } while (changePriorities(bad_i) && !solution_found && (sresult.agentFate == "survived"));

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
    sresult.timingInformation = timer.elapsed_time_csv();
    return sresult;
}

std::unordered_map<std::pair<int, int>, double,
                   boost::hash<std::pair<int, int>>>
                                                        RTNode::_static_h;
std::unordered_map<RTNode, double, boost::hash<RTNode>> RTNode::_dynamic_h;
std::unordered_multimap<RTNode, RTNode, boost::hash<RTNode>> RTNode::parents;
int                                                     RTNode::dynmode = 0;
std::string RTNode::expansionOrderStr = "Not Set";

bool Realtime_SIPP::findPath(unsigned int numOfCurAgent, const Map& map)
{
    double prior_g = 0.0;

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
    compute_static_h(map);
    constraints->use_likhachev = config->use_likhachev;
    RTResultPathInfo resultPath;
    timer.resume_si();
    constraints->resetSafeIntervals(map.width, map.height);
    constraints->updateCellSafeIntervals({curagent.start_i, curagent.start_j});
    timer.stop_si();
    RTNode curNode(curagent.start_i, curagent.start_j, -1);
    curNode.set_zero();
    RTNode goalNode(curagent.goal_i, curagent.goal_j, -1);
    goalNode.set_inf();
    // curNode.F           = getHValue(curNode.i, curNode.j);
    //curNode.set_static_h(curNode.static_h());

    curNode.interval    = constraints->getSafeInterval(curNode.i, curNode.j, 0);
    curNode.interval_id = curNode.interval.id;
    curNode.heading     = curagent.start_heading;
    curNode.optimal     = true;
    addOpen(curNode);
    int reexpanded(0);
    // int             close_id(0);
    std::list<RTNode> reexpanded_list;
    // real-time search loop
    //int iterationCounter(0);
    hppath.push_back(curNode);
    lppath.push_back(curNode);
    //while (iterationCounter++ < 10000) {
    while (true){
        // DEBUG_MSG_NO_LINE_BREAK( "iteration id " << iterationCounter);
        //DEBUG_MSG("iteration id " << iterationCounter);

        // Tianyi note: this goal test might be too much simplified, check
        // AA_SIPP::stpCriterion
        prior_g = curNode.g();
        debug_h(curNode, map);
        if (curNode.i == curagent.goal_i && curNode.j == curagent.goal_j) {
            DEBUG_MSG("goal reached yay!");
            if (curNode.Parent){
                hppath.push_back(*curNode.Parent);
                lppath.push_back(*curNode.Parent);
            }
            //hppath.push_back(curNode);
            //lppath.push_back(curNode);      
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
            resultPath.pathlength      = curNode.g();
            resultPath.reopened        = constraints->reopened; //;
            resultPath.reexpanded      = reexpanded;
            resultPath.reexpanded_list = reexpanded_list;
            sresult.pathfound          = true;
            sresult.flowtime += curNode.g();
            sresult.makespan = std::max(sresult.makespan, curNode.g());
            sresult.pathInfo[numOfCurAgent] = resultPath;
            sresult.agentsSolved++;
            break;
        }
        if ((timer.elapsed_s() > config->timelimit) || (sresult.steps > config->steplimit)){
            DEBUG_MSG("CPU Time limit reached");
            sresult.agentFate = "timed out";
            break;
        }

        timeval beginOfRealtimeCycle, endOfRealtimeCycle;
        gettimeofday(&beginOfRealtimeCycle, NULL);
        timer.resume_expansion();
        expansionModulePtr->runSearch(curNode, goalNode, map, close, reexpanded,
                                      reexpanded_list, this);
        if (false && curNode.i == curagent.goal_i && curNode.j == curagent.goal_j) {
            continue;
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
            resultPath.pathlength      = curNode.g();
            resultPath.reopened        = constraints->reopened; //;
            resultPath.reexpanded      = reexpanded;
            resultPath.reexpanded_list = reexpanded_list;
            sresult.pathfound          = true;
            sresult.flowtime += curNode.g();
            sresult.makespan = std::max(sresult.makespan, curNode.g());
            sresult.pathInfo[numOfCurAgent] = resultPath;
            sresult.agentsSolved++;
            break;
        }
        timer.stop_expansion();
        gettimeofday(&endOfRealtimeCycle, nullptr);
        if (open.empty()) {
            DEBUG_MSG("Break lookahead, OPEN list is empty! ");
            if (curNode.interval.end == CN_INFINITY){
                sresult.agentFate = "trapped";
            }
            else{
                sresult.agentFate = "died";
            }
            break;
        }
        // learning phase
        // update the heuristic in closed list
        timer.resume_learning();
        learningModulePtr->learn(open, close);
        timer.stop_learning();

        RTNode::clear_parents();
        // decision-making phase
        // 1) commit the the toplevel action that would lead to the best search
        // frontier node
        //
        timer.resume_decision();
        auto bestTLA = decisionModulePtr->backupAndRecordPartialPlan(
          curNode, beginOfRealtimeCycle, endOfRealtimeCycle, curagent.goal_i,
          curagent.goal_j, hppath, lppath, this);
        timer.stop_decision();
        if (config->issituated){
            lookaheadBudget = (int)std::max(1.0, std::floor((bestTLA.g() - prior_g)*config->fixedlookahead));
         
        }
        // 2) re-root the search tree to the best successor node
        curNode = bestTLA;
        ++(sresult.steps);
        curNode.set_parent(nullptr);
        open.clear();
        curNode.prune_past();
        addOpen(curNode);
        // curExpansion = 0;
        RTNode resetGoalNode(curagent.goal_i, curagent.goal_j, -1);
        resetGoalNode.set_inf();
        goalNode = resetGoalNode;
        close.clear();
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
    debug_h_to_file("debug_h.json");
    return resultPath.pathfound;
}

void Realtime_SIPP::recordToOnlinePath(const RTNode&  rootNode,
                                       const RTNode&  frontierNode,
                                       const timeval& begin, const timeval& end)
{
    auto curNode = frontierNode;

    std::list<RTNode>   path;
    std::vector<RTNode> sections;
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
        RTNode add = sections.back();
        DEBUG_MSG_RED("DO NOT REACH");
        add.heading = curagent.goal_heading;
        sections.back().set_static_g(
          sections.back().g() -
          getRCost(sections.back().heading, curagent.goal_heading));
        sections.push_back(add);
    }
    /*
    // Something is broken here, not clear the purpose of this
    for (unsigned int i = 1; i < sections.size(); i++) {
        double wait_duration = (sections[i].g() - (sections[i - 1].g() + getCost(sections[i].i, sections[i].j, sections[i - 1].i, sections[i - 1].j))) / curagent.mspeed;
        if (wait_duration > CN_EPSILON) {
            // need to insert a wait
            RTNode add = sections[i - 1];
            add.Parent = sections[i].Parent;
            add.set_dynamic_g();
            add.heading = sections[i].heading;
            sections.emplace(sections.begin() + i, add);
            i++;
        }
    }
    */ 
    if (config->planforturns && curagent.goal_heading >= 0) {
        sections.pop_back();
    }

    RTResultPathInfo partialPath;
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
    partialPath.pathlength = frontierNode.g() - rootNode.g();
    onlinePlanSections.push_back(partialPath);
}

void Realtime_SIPP::addOpen(RTNode& newNode)
{
    auto range     = open.get<1>().equal_range(boost::make_tuple<int, int, int>(
      newNode.i, newNode.j, newNode.interval_id));
    auto it        = range.first;
    bool dominated = false;
    // newNode.open_id = open_id;
    // open_id++;
    while (it != range.second) {
        if (it->optimal != newNode.optimal) {
            it++;
            continue;
        }
        if ((it->g() - newNode.g() + getRCost(it->heading, newNode.heading)) <
            CN_EPSILON) // if existing state dominates new one
        {
            dominated = true;
        } else if ((newNode.g() + getRCost(it->heading, newNode.heading) -
                    it->g()) < CN_EPSILON) {
            open.get<1>().erase(it);
            range = open.get<1>().equal_range(boost::make_tuple<int, int, int>(
              newNode.i, newNode.j, newNode.interval_id));
            it    = range.first;
            continue;
            break;
        }
        it++;
    }
    if (!dominated) {
        open.insert(newNode);
        if (config->use_focal) {
            if (open.get<0>().begin()->F() * config->focal_weight >
                newNode.F() - CN_EPSILON) {
                Focal_Elem elem(newNode.g(), newNode.F());
                elem.leaps = focal_heuristic.get_value(newNode.i, newNode.j,
                                                       curagent.id_num);
                focal.insert(elem);
            }
        }
    }
}

RTNode Realtime_SIPP::findMin()
{
    if (!config->use_focal) {
        auto node = *open.get<0>().begin();
        open.get<0>().erase(open.get<0>().begin());
        return node;
    }
    double cost = open.get<0>().begin()->F();
    if (focal.empty())
        update_focal(cost);
    auto pointer = open.get<2>().find(focal.get<0>().begin()->open_id);
    while (pointer == open.get<2>().end()) {
        focal.get<0>().erase(focal.get<0>().begin());
        if (focal.empty())
            update_focal(cost);
        pointer = open.get<2>().find(focal.get<0>().begin()->open_id);
    }
    auto min = *pointer;
    open.get<2>().erase(pointer);
    if (open.get<0>().begin()->F() > cost + CN_EPSILON)
        update_focal(cost);
    return min;
}

bool Realtime_SIPP::stopCriterion(const RTNode& curNode, RTNode& goalNode)
{
    if (curNode.i == curagent.goal_i && curNode.j == curagent.goal_j &&
        curNode.interval.end == CN_INFINITY) {
        if (!config->planforturns ||
            curagent.goal_heading == CN_HEADING_WHATEVER) {
            goalNode = curNode;
        } else if (goalNode.g() >
                   curNode.g() +
                     getRCost(curNode.heading, curagent.goal_heading)) {
            goalNode = curNode;
            goalNode.set_static_g(
              curNode.static_g() +
              getRCost(curNode.heading, curagent.goal_heading));
            goalNode.set_dynamic_g(curNode.dynamic_g());
        }
    }
    if (goalNode.F() - CN_EPSILON < curNode.F() ||
        (goalNode.g() < CN_INFINITY &&
         fabs(goalNode.g() - goalNode.interval.begin) < CN_EPSILON)) {
        return true;
    }
    if (open.empty()) {
        // std::cout << "OPEN list is empty! ";
        if (curNode.interval.end == CN_INFINITY){
            sresult.agentFate = "trapped";
        }
        else{
            sresult.agentFate = "died";
        }
        return true;
    }
    return false;
}

double Realtime_SIPP::calcHeading(const RTNode& node, const RTNode& son)
{
    double heading =
      acos((son.j - node.j) / getCost(son.i, son.j, node.i, node.j)) * 180 / PI;
    if (node.i < son.i) {
        heading = 360 - heading;
    }
    return heading;
}

std::list<RTNode> Realtime_SIPP::findSuccessors(const RTNode curNode,
                                                const Map&   map)
{
    if (config->isUnitWaitRepresentation) {
        return findSuccessorsUsingUnitWaitRepresentation(curNode, map);
    }

    RTNode                    newNode;
    RTNode                    angleNode;
    std::list<RTNode>         successors;
    std::vector<double>       EAT;
    std::vector<SafeInterval> intervals;
    // double                    h_value;
    auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
    std::vector<RTNode> moves = map.getValidRTMoves(
      curNode.i, curNode.j, config->connectedness, curagent.size);
    for (auto m : moves) {
        if (lineofsight.checkTraversability(curNode.i + m.i, curNode.j + m.j,
                                            map)) {
            newNode.i          = curNode.i + m.i;
            newNode.j          = curNode.j + m.j;
            newNode.heading_id = m.heading_id;
            timer.stop_expansion();
            timer.resume_si();
            constraints->updateCellSafeIntervals({newNode.i, newNode.j});
            timer.stop_si();
            timer.resume_expansion();
            newNode.heading = calcHeading(curNode, newNode);
            angleNode = curNode; // the same state, but with extended g-value

            angleNode.set_static_g(
              angleNode.static_g() +
              getRCost(angleNode.heading, newNode.heading) +
              config->additionalwait);
            newNode.set_static_g(angleNode.static_g() +
                                 m.g() / curagent.mspeed);
            newNode.set_dynamic_g(angleNode.dynamic_g());
            newNode.set_parent(&angleNode);
            newNode.optimal = curNode.optimal;
            //newNode.set_static_h(config->h_weight * getHValue(newNode.i, newNode.j));
            if (angleNode.g() <= angleNode.interval.end) { //something seems off
                timer.stop_expansion();
                timer.resume_si();
                intervals =
                  constraints->findIntervals(newNode, EAT, close, map);
                timer.stop_si();
                timer.resume_expansion();
                unsigned long num_of_intervals =
                  std::min(config->maxNumOfIntervalsPerMove, intervals.size());
                for (unsigned int k = 0; k < num_of_intervals; k++) {
                    newNode.interval = intervals[k];
                    newNode.set_parent(parent);
                    newNode.set_static_g(newNode.Parent->static_g() +
                                         getCost(newNode.Parent->i,
                                                 newNode.Parent->j, newNode.i,
                                                 newNode.j) /
                                           curagent.mspeed);
                    newNode.set_dynamic_g(EAT[k] - newNode.static_g());
                    newNode.interval_id = newNode.interval.id;
                    successors.push_front(newNode);
                }
            }
            if (config->allowanyangle) {
                std::cerr << "Please disable allowanyangle in cofig\n";
                exit(1);
            }
        }
    }

    return successors;
}

std::list<RTNode> Realtime_SIPP::findSuccessorsUsingUnitWaitRepresentation(
  const RTNode curNode, const Map& map)
{
    RTNode                    newNode;
    RTNode                    angleNode;
    RTNode                    newNodeWithWait;
    RTNode                    angleNodeWithWait;
    std::list<RTNode>         successors;
    std::vector<double>       EAT;
    std::vector<SafeInterval> intervals;
    // double                    h_value;
    auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
    std::vector<RTNode> moves = map.getValidRTMoves(
      curNode.i, curNode.j, config->connectedness, curagent.size);
    for (auto m : moves) {
        if (lineofsight.checkTraversability(curNode.i + m.i, curNode.j + m.j,
                                            map)) {
            newNode.i          = curNode.i + m.i;
            newNode.j          = curNode.j + m.j;
            newNode.heading_id = m.heading_id;

            newNodeWithWait.i          = curNode.i + m.i;
            newNodeWithWait.j          = curNode.j + m.j;
            newNodeWithWait.heading_id = m.heading_id;
            timer.stop_expansion();
            timer.resume_si();
            constraints->updateCellSafeIntervals({newNode.i, newNode.j});
            timer.stop_si();
            timer.resume_expansion();
            newNode.heading         = calcHeading(curNode, newNode);
            newNodeWithWait.heading = calcHeading(curNode, newNodeWithWait);
            angleNode = curNode; // the same state, but with extended g-value
            angleNodeWithWait =
              curNode; // the same state, but with extended g-value

            // angleNode.debug();
            // m.debug();

            angleNode.set_static_g(
              angleNode.static_g() +
              getRCost(angleNode.heading, newNode.heading));
            angleNodeWithWait.set_static_g(
              angleNode.static_g() +
              getRCost(angleNode.heading, newNode.heading) +
              config->unitWaitDuration);
            // angleNode.debug();

            newNode.set_static_g(angleNode.static_g() +
                                 m.g() / curagent.mspeed);
            newNode.set_dynamic_g(angleNode.dynamic_g());
            newNode.set_parent(&angleNode);
            newNode.optimal = curNode.optimal;
            //newNode.set_static_h(config->h_weight * getHValue(newNode.i, newNode.j));
            // newNode.debug();
            newNodeWithWait.set_static_g(angleNodeWithWait.static_g() +
                                         m.g() / curagent.mspeed);
            newNodeWithWait.set_dynamic_g(angleNodeWithWait.dynamic_g());
            newNodeWithWait.set_parent(&angleNodeWithWait);
            newNodeWithWait.optimal = curNode.optimal;
            //newNodeWithWait.set_static_h(config->h_weight * getHValue(newNodeWithWait.i, newNodeWithWait.j));

            if (angleNode.g() <= angleNode.interval.end) {
                timer.stop_expansion();
                timer.resume_si();
                intervals =
                  constraints->findIntervals(newNode, EAT, close, map);
                timer.stop_si();
                timer.resume_expansion();
                if (intervals.size() > 0) {
                    newNode.interval = intervals[0];
                    newNode.set_parent(parent);
                    newNode.set_static_g(newNode.Parent->static_g() +
                                         getCost(newNode.Parent->i,
                                                 newNode.Parent->j, newNode.i,
                                                 newNode.j) /
                                           curagent.mspeed);
                    newNode.set_dynamic_g(EAT[0] - newNode.static_g());
                    newNode.interval_id = newNode.interval.id;
                    successors.push_front(newNode);
                }
            }

            if (angleNodeWithWait.g() <= angleNodeWithWait.interval.end) {
                timer.stop_expansion();
                timer.resume_si();
                intervals =
                  constraints->findIntervals(newNodeWithWait, EAT, close, map);
                timer.stop_si();
                timer.resume_expansion();
                if (intervals.size() > 0) {
                    newNodeWithWait.interval = intervals[0];
                    newNodeWithWait.set_parent(parent);
                    newNodeWithWait.set_static_g(newNodeWithWait.Parent->static_g() +
                                         getCost(newNodeWithWait.Parent->i,
                                                 newNodeWithWait.Parent->j, newNodeWithWait.i,
                                                 newNodeWithWait.j) /
                                           curagent.mspeed);
                    newNodeWithWait.set_dynamic_g(EAT[0] - newNodeWithWait.static_g());
                    newNodeWithWait.interval_id = newNodeWithWait.interval.id;
                    successors.push_front(newNodeWithWait);
                }
            }

            if (config->allowanyangle) {
                std::cerr << "Please disable allowanyangle in cofig\n";
                exit(1);
            }
        }
    }

    return successors;
}

void Realtime_SIPP::makePrimaryPath(RTNode curNode)
{
    hppath.clear();
    hppath.shrink_to_fit();
    std::list<RTNode> path;
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
        RTNode add  = hppath.back();
        add.heading = curagent.goal_heading;
        hppath.back().set_static_g(
          hppath.back().static_g() -
          getRCost(hppath.back().heading, curagent.goal_heading));
        hppath.push_back(add);
    }
    for (unsigned int i = 1; i < hppath.size(); i++) {
        if ((hppath[i].g() -
             (hppath[i - 1].g() + getCost(hppath[i].i, hppath[i].j,
                                          hppath[i - 1].i, hppath[i - 1].j) /
                                    curagent.mspeed)) > CN_EPSILON) {
            RTNode add = hppath[i - 1];
            add.set_parent(hppath[i].Parent);
            add.set_static_g(hppath[i].static_g() - getCost(hppath[i].i, hppath[i].j, hppath[i - 1].i,
                                     hppath[i - 1].j) /
                               curagent.mspeed);
            add.heading = hppath[i].heading;
            hppath.emplace(hppath.begin() + i, add);
            i++;
        }
    }
    if (config->planforturns && curagent.goal_heading >= 0) {
        hppath.pop_back();
    }
}

void Realtime_SIPP::makeSecondaryPath(RTNode curNode)
{
    lppath.clear();
    if (curNode.Parent != nullptr) {
        std::vector<RTNode> lineSegment;
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

void Realtime_SIPP::calculateLineSegment(std::vector<RTNode>& line,
                                         const RTNode&        start,
                                         const RTNode&        goal)
{
    int i1 = start.i;
    int i2 = goal.i;
    int j1 = start.j;
    int j2 = goal.j;

    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i  = (i1 < i2 ? 1 : -1);
    int step_j  = (j1 < j2 ? 1 : -1);
    int error   = 0;
    int i       = i1;
    int j       = j1;
    if (delta_i > delta_j) {
        for (; i != i2; i += step_i) {
            line.push_back(RTNode(i, j));
            error += delta_j;
            if ((error << 1) > delta_i) {
                j += step_j;
                error -= delta_i;
            }
        }
    } else {
        for (; j != j2; j += step_j) {
            line.push_back(RTNode(i, j));
            error += delta_i;
            if ((error << 1) > delta_j) {
                i += step_i;
                error -= delta_j;
            }
        }
    }
}

RTNode Realtime_SIPP::resetParent(RTNode current, RTNode Parent, const Map& map)
{
    if (Parent.Parent == nullptr ||
        (current.i == Parent.Parent->i && current.j == Parent.Parent->j)) {
        return current;
    }
    if (lineofsight.checkLine(Parent.Parent->i, Parent.Parent->j, current.i,
                              current.j, map)) {
        current.set_static_g(
          Parent.Parent->static_g() +
          getCost(Parent.Parent->i, Parent.Parent->j, current.i, current.j) /
            curagent.mspeed);
        current.set_dynamic_g(Parent.Parent->dynamic_g());
        current.set_parent(Parent.Parent);
    }
    return current;
}

std::vector<conflict> Realtime_SIPP::CheckConflicts(const Task& task)
{
    std::vector<conflict>              conflicts(0);
    conflict                           conf;
    RTNode                             cur, check;
    std::vector<std::vector<conflict>> positions;
    positions.resize(sresult.agents);
    for (unsigned int i = 0; i < sresult.agents; i++) {
        if (!sresult.pathInfo[i].pathfound) {
            continue;
        }
        positions[i].resize(0);
        int    k    = 0;
        double part = 1;
        for (unsigned int j = 1; j < sresult.pathInfo[i].sections.size(); j++) {
            cur          = sresult.pathInfo[i].sections[j];
            check        = sresult.pathInfo[i].sections[j - 1];
            int    di    = cur.i - check.i;
            int    dj    = cur.j - check.j;
            double dist  = (cur.g() - check.g()) * 10;
            int    steps = (cur.g() - check.g()) * 10;
            if (dist - steps + part >= 1) {
                steps++;
                part = dist - steps;
            } else {
                part += dist - steps;
            }
            double stepi = double(di) / dist;
            double stepj = double(dj) / dist;
            double curg  = double(k) * 0.1;
            double curi =
              check.i + (curg - check.g()) * di / (cur.g() - check.g());
            double curj =
              check.j + (curg - check.g()) * dj / (cur.g() - check.g());
            conf.i = curi;
            conf.j = curj;
            conf.g = curg;
            if (curg <= cur.g()) {
                positions[i].push_back(conf);
                k++;
            }
            while (curg <= cur.g()) {
                if (curg + 0.1 > cur.g()) {
                    break;
                }
                curi += stepi;
                curj += stepj;
                curg += 0.1;
                conf.i = curi;
                conf.j = curj;
                conf.g = curg;
                positions[i].push_back(conf);
                k++;
            }
        }
        if (double(k - 1) * 0.1 < sresult.pathInfo[i].sections.back().g()) {
            conf.i = sresult.pathInfo[i].sections.back().i;
            conf.j = sresult.pathInfo[i].sections.back().j;
            conf.g = sresult.pathInfo[i].sections.back().g();
            positions[i].push_back(conf);
        }
    }
    unsigned int max     = 0;
    double       sumsize = 0;
    for (unsigned int i = 0; i < positions.size(); i++) {
        if (positions[i].size() > max) {
            max = positions[i].size();
        }
    }
    for (unsigned int i = 0; i < sresult.agents; i++) {
        for (unsigned int k = 0; k < max; k++) {
            for (unsigned int j = i + 1; j < sresult.agents; j++) {
                if (!sresult.pathInfo[j].pathfound ||
                    !sresult.pathInfo[i].pathfound) {
                    continue;
                }
                sumsize = task.getAgent(i).size + task.getAgent(j).size;
                conflict a, b;
                if (positions[i].size() > k) {
                    a = positions[i][k];
                } else {
                    a = positions[i].back();
                }
                if (positions[j].size() > k) {
                    b = positions[j][k];
                } else {
                    b = positions[j].back();
                }
                if (sqrt((a.i - b.i) * (a.i - b.i) +
                         (a.j - b.j) * (a.j - b.j)) +
                      CN_EPSILON <
                    sumsize) {
                    std::cout << i << " " << j << " " << a.i << " " << a.j
                              << " " << b.i << " " << b.j << " "
                              << sqrt((a.i - b.i) * (a.i - b.i) +
                                      (a.j - b.j) * (a.j - b.j))
                              << "\n";
                    conf.i      = b.i;
                    conf.j      = b.j;
                    conf.agent1 = i;
                    conf.agent2 = j;
                    conf.g      = b.g;
                    conflicts.push_back(conf);
                }
            }
        }
    }
    return conflicts;
}

void Realtime_SIPP::update_focal(double cost)
{
    for (auto it = open.get<0>().begin(); it != open.get<0>().end(); it++) {
        if (it->F() > cost * config->focal_weight) {
            break;
        }
        focal.insert(
          Focal_Elem(it->g(), it->F(),
                     focal_heuristic.get_value(it->i, it->j, curagent.id_num)));
    }
}

void Realtime_SIPP::compute_static_h(const Map& map){
    for (int column = 0; column < map.width; column++){
        for (int row = 0; row < map.height; row++){
            RTNode node = RTNode(row, column);
            node.set_static_h(getHValue(row, column));      
        }
    }
}

#ifdef DEBUG
    void Realtime_SIPP::debug_h(const RTNode& curNode, const Map& map){
        std::vector<double> static_h;
        static_h.resize(map.width*map.height);
        for (int width = 0; width < map.width; width++){
            for (int height = 0; height < map.height; height++){
                size_t ind = width + map.width*height;
                RTNode node = RTNode(height, width);
                static_h[ind] = node.static_h();
            } 
        }    
        hjson[std::to_string(curNode.g()) + ":static_h"] = static_h; 
    }

    void Realtime_SIPP::debug_h_to_file(const std::string & filename){
        std::ofstream o(filename);
        o << std::setw(4) << hjson << std::endl;
    }
#else
    void Realtime_SIPP::debug_h(const RTNode& curNode, const Map& map){
        while(false){
            (void)(curNode);
            (void)(map);
        }
    }
    void Realtime_SIPP::debug_h_to_file(const std::string & filename){
        while(false){
            (void)(filename);
        };
    }
#endif