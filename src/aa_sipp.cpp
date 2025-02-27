#include "aa_sipp.h"
#include "debug.h"
#include "structs.h"

AA_SIPP::AA_SIPP(const Config& config)
{
    this->config = std::make_shared<const Config>(config);
    constraints  = nullptr;
    open_id      = 0;
}

AA_SIPP::~AA_SIPP() = default;

auto AA_SIPP::stopCriterion(const Node& curNode, Node& goalNode) -> bool
{
    if (open.empty()) {
        DEBUG_MSG("Break lookahead, OPEN list is empty! ");
        if (curNode.interval.end == CN_INFINITY){
            sresult.agentFate = "trapped";
        }
        else{
            sresult.agentFate = "died";
        }
        return true;
    }
    if (curNode.i == curagent.goal_i && curNode.j == curagent.goal_j &&
        curNode.interval.end == CN_INFINITY) {
        if (!config->planforturns ||
            curagent.goal_heading == CN_HEADING_WHATEVER) {
            goalNode = curNode;
        } else if (goalNode.g > curNode.g + getRCost(curNode.heading,
                                                     curagent.goal_heading)) {
            goalNode = curNode;
            goalNode.g =
              curNode.g + getRCost(curNode.heading, curagent.goal_heading);
            goalNode.F =
              curNode.F + getRCost(curNode.heading, curagent.goal_heading);
        }
    }
    if (goalNode.F - CN_EPSILON < curNode.F ||
        (goalNode.g < CN_INFINITY &&
         fabs(goalNode.g - goalNode.interval.begin) < CN_EPSILON)) {
        return true;
    }
    return false;
}

auto AA_SIPP::getCost(int a_i, int a_j, int b_i, int b_j) -> double
{
    return sqrt((a_i - b_i) * (a_i - b_i) + (a_j - b_j) * (a_j - b_j));
}

auto AA_SIPP::getHValue(int i, int j) -> double
{
    if (config->allowanyangle || config->connectedness > 3) { // euclid
        return (sqrt(pow(i - curagent.goal_i, 2) +
                     pow(j - curagent.goal_j, 2))) /
               curagent.mspeed;
    } else if (config->connectedness == 2) { // manhattan
        return (abs(i - curagent.goal_i) + abs(j - curagent.goal_j)) /
               curagent.mspeed;
    } else { // k=3, use diagonal
        return (abs(abs(i - curagent.goal_i) - abs(j - curagent.goal_j)) +
                sqrt(2.0) * std::min(abs(i - curagent.goal_i),
                                     abs(j - curagent.goal_j))) /
               curagent.mspeed;
    }
}

auto AA_SIPP::getRCost(double headingA, double headingB) -> double
{
    if (config->planforturns) {
        return std::min(360 - fabs(headingA - headingB),
                        fabs(headingA - headingB)) /
               (curagent.rspeed * 180.0);
    }
    return 0;
}

auto AA_SIPP::calcHeading(const Node& node, const Node& son) -> double
{
    double heading =
      acos((son.j - node.j) / getCost(son.i, son.j, node.i, node.j)) * 180 / PI;
    if (node.i < son.i) {
        heading = 360 - heading;
    }
    return heading;
}

std::list<Node> AA_SIPP::findSuccessors(const Node curNode, const Map& map)
{
    Node                      newNode;
    Node                      angleNode;
    std::list<Node>           successors;
    std::vector<double>       EAT;
    std::vector<SafeInterval> intervals;
    double                    h_value;
    auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
    std::vector<Node> moves = map.getValidMoves(
      curNode.i, curNode.j, config->connectedness, curagent.size);
    for (auto m : moves) {
        if (lineofsight.checkTraversability(curNode.i + m.i, curNode.j + m.j,
                                            map)) {
            newNode.i          = curNode.i + m.i;
            newNode.j          = curNode.j + m.j;
            newNode.heading_id = m.heading_id;
            constraints->updateCellSafeIntervals({newNode.i, newNode.j});
            newNode.heading = calcHeading(curNode, newNode);
            angleNode       = curNode; // the same state, but with extended g-value
            angleNode.g +=
              getRCost(angleNode.heading, newNode.heading) +
              config->additionalwait; // to compensate the amount of time
                                      // required for rotation
            newNode.g       = angleNode.g + m.g / curagent.mspeed;
            newNode.Parent  = &angleNode;
            newNode.optimal = curNode.optimal;
            h_value = config->h_weight * getHValue(newNode.i, newNode.j);

            if (angleNode.g <= angleNode.interval.end) {
                intervals =
                  constraints->findIntervals(newNode, EAT, close, map);
                for (unsigned int k = 0; k < intervals.size(); k++) {
                    newNode.interval    = intervals[k];
                    newNode.Parent      = parent;
                    newNode.g           = EAT[k];
                    newNode.interval_id = newNode.interval.id;
                    newNode.F           = newNode.g + h_value;
                    successors.push_front(newNode);
                }
            }
            if (config->allowanyangle) {
                newNode = resetParent(newNode, curNode, map);
                if (newNode.Parent->i != parent->i ||
                    newNode.Parent->j != parent->j) {
                    angleNode       = *newNode.Parent;
                    newNode.heading = calcHeading(
                      *newNode.Parent,
                      newNode); // new heading with respect to new parent
                    angleNode.g +=
                      getRCost(angleNode.heading, newNode.heading) +
                      config->additionalwait; // count new additional time
                                              // required for rotation
                    newNode.g += getRCost(angleNode.heading, newNode.heading) +
                                 config->additionalwait;
                    newNode.Parent = &angleNode;
                    if (angleNode.g > angleNode.interval.end) {
                        continue;
                    }
                    intervals =
                      constraints->findIntervals(newNode, EAT, close, map);
                    for (unsigned int k = 0; k < intervals.size(); k++) {
                        newNode.interval    = intervals[k];
                        newNode.Parent      = parent->Parent;
                        newNode.g           = EAT[k];
                        newNode.interval_id = newNode.interval.id;
                        newNode.F           = newNode.g + h_value;
                        successors.push_front(newNode);
                    }
                }
            }
        }
    }

    return successors;
}

void AA_SIPP::update_focal(double cost)
{
    for (auto it = open.get<0>().begin(); it != open.get<0>().end(); it++) {
        if (it->F > cost * config->focal_weight) {
            break;
        }
        focal.insert(Focal_Elem(
          *it, focal_heuristic.get_value(it->i, it->j, curagent.id_num)));
    }
}

Node AA_SIPP::findMin()
{
    if (!config->use_focal) {
        auto node = *open.get<0>().begin();
        open.get<0>().erase(open.get<0>().begin());
        return node;
    }
    double cost = open.get<0>().begin()->F;
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
    DEBUG_MSG_RED("Open Size");
    DEBUG_MSG_RED(open.size());
    if ((open.size() > 0) && (open.get<0>().begin()->F > cost + CN_EPSILON))
        update_focal(cost);
    return min;
}

void AA_SIPP::addOpen(Node& newNode)
{
    DEBUG_MSG_RED("adding:");
    newNode.debug();
    auto range     = open.get<1>().equal_range(boost::make_tuple<int, int, int>(
      newNode.i, newNode.j, newNode.interval_id));
    auto it        = range.first;
    bool dominated = false;
    newNode.open_id = open_id;
    open_id++;
    while (it != range.second) {
        if (it->optimal != newNode.optimal) {
            it++;
            continue;
        }
        if ((it->g - newNode.g + getRCost(it->heading, newNode.heading)) <
            CN_EPSILON) // if existing state dominates new one
        {
            dominated = true;
        } else if ((newNode.g + getRCost(it->heading, newNode.heading) -
                    it->g) < CN_EPSILON) {
            DEBUG_MSG_NO_LINE_BREAK_RED("Dominates: ");
            it->debug();
            open.get<1>().erase(it);
            range = open.get<1>().equal_range(boost::make_tuple<int, int, int>(
              newNode.i, newNode.j, newNode.interval_id));
            it    = range.first;
            continue;
            break;
        }
        it++;
    }
    DEBUG_MSG_NO_LINE_BREAK_RED("Dominated: ");
    DEBUG_MSG_RED(dominated);
    if (!dominated) {
        open.insert(newNode);
        if (config->use_focal) {
            if (open.get<0>().begin()->F * config->focal_weight >
                newNode.F - CN_EPSILON) {
                Focal_Elem elem(newNode);
                elem.leaps = focal_heuristic.get_value(newNode.i, newNode.j,
                                                       curagent.id_num);
                focal.insert(elem);
            }
        }
    }
}

void AA_SIPP::setPriorities(const Task& task)
{
    current_priorities.clear();
    current_priorities.resize(task.getNumberOfAgents(), -1);
    if (config->initialprioritization == CN_IP_FIFO) {
        for (int i = 0; i < task.getNumberOfAgents(); i++) {
            current_priorities[i] = i;
        }
    } else if (config->initialprioritization != CN_IP_RANDOM) {
        std::vector<double> dists(task.getNumberOfAgents(), -1);
        for (int i = 0; i < task.getNumberOfAgents(); i++) {
            dists[i] =
              sqrt(pow(task.getAgent(i).start_i - task.getAgent(i).goal_i, 2) +
                   pow(task.getAgent(i).start_j - task.getAgent(i).goal_j, 2));
        }
        int k = task.getNumberOfAgents() - 1;
        while (k >= 0) {
            double mindist = CN_INFINITY;
            int    min_i   = -1;
            for (unsigned int i = 0; i < dists.size(); i++) {
                if (mindist > dists[i]) {
                    min_i   = i;
                    mindist = dists[i];
                }
            }
            if (config->initialprioritization == CN_IP_LONGESTF) {
                current_priorities[k] = min_i;
            } else {
                current_priorities[task.getNumberOfAgents() - k - 1] = min_i;
            }
            dists[min_i] = CN_INFINITY;
            k--;
        }
    } else // random
    {
        for (int i = 0; i < task.getNumberOfAgents(); i++) {
            current_priorities[i] = i;
        }
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(), current_priorities.end(), g);
    }
}

bool AA_SIPP::changePriorities(int bad_i)
{
    if (config->rescheduling == CN_RE_NO) {
        return false;
    }
    priorities.push_back(current_priorities);
    if (config->rescheduling ==
        CN_RE_RULED) // rises the piority of the agent that can't find its path
    {
        for (auto it = current_priorities.begin();
             it != current_priorities.end(); it++) {
            if (*it == bad_i) {
                current_priorities.erase(it);
                current_priorities.insert(current_priorities.begin(), bad_i);
                break;
            }
        }
        for (unsigned int i = 0; i < priorities.size(); i++) {
            for (unsigned int j = 0; j < priorities[i].size(); j++) {
                if (j + 1 == priorities[i].size()) {
                    return false;
                }
                if (current_priorities[j] != priorities[i][j]) {
                    break;
                }
            }
        }
        return true;
    } else // random
    {
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(), current_priorities.end(), g);
        bool unique = false;
        int  maxtries(1000000);
        int  tries(0);
        while (!unique && tries < maxtries) {
            tries++;
            for (unsigned int i = 0; i < priorities.size(); i++) {
                for (unsigned int j = 0; j < priorities[i].size(); j++) {
                    if (j + 1 == priorities[i].size()) {
                        unique = false;
                    }
                    if (current_priorities[j] != priorities[i][j]) {
                        break;
                    }
                }
                if (!unique) {
                    std::shuffle(current_priorities.begin(),
                                 current_priorities.end(), g);
                    break;
                }
            }
            unique = true;
        }
        return unique;
    }
}

SearchResult AA_SIPP::startSearch(Map& map, Task& task,
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
    return sresult;
}

auto AA_SIPP::resetParent(Node current, Node Parent, const Map& map) -> Node
{
    if (Parent.Parent == nullptr ||
        (current.i == Parent.Parent->i && current.j == Parent.Parent->j)) {
        return current;
    }
    if (lineofsight.checkLine(Parent.Parent->i, Parent.Parent->j, current.i,
                              current.j, map)) {
        current.g =
          Parent.Parent->g +
          getCost(Parent.Parent->i, Parent.Parent->j, current.i, current.j) /
            curagent.mspeed;
        current.Parent = Parent.Parent;
    }
    return current;
}

bool AA_SIPP::findPath(unsigned int numOfCurAgent, const Map& map)
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
    DEBUG_MSG_RED("Reach findPath");
    constraints->use_likhachev = config->use_likhachev;
    ResultPathInfo resultPath;
    constraints->resetSafeIntervals(map.width, map.height);
    //constraints->debug_safe_intervals();
    constraints->updateCellSafeIntervals({curagent.start_i, curagent.start_j});
    //constraints->debug_safe_intervals();
    Node curNode(curagent.start_i, curagent.start_j, -1, 0, 0),
      goalNode(curagent.goal_i, curagent.goal_j, -1, CN_INFINITY, CN_INFINITY);
    curNode.F           = getHValue(curNode.i, curNode.j);
    curNode.interval    = constraints->getSafeInterval(curNode.i, curNode.j, 0);
    curNode.interval_id = curNode.interval.id;
    curNode.heading     = curagent.start_heading;
    curNode.optimal     = true;
    addOpen(curNode);
    debug_open(open);
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
            debug_open(open);
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

std::vector<conflict> AA_SIPP::CheckConflicts(const Task& task)
{
    std::vector<conflict>              conflicts(0);
    conflict                           conf;
    Node                               cur, check;
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
            double dist  = (cur.g - check.g) * 10;
            int    steps = (cur.g - check.g) * 10;
            if (dist - steps + part >= 1) {
                steps++;
                part = dist - steps;
            } else {
                part += dist - steps;
            }
            double stepi = double(di) / dist;
            double stepj = double(dj) / dist;
            double curg  = double(k) * 0.1;
            double curi  = check.i + (curg - check.g) * di / (cur.g - check.g);
            double curj  = check.j + (curg - check.g) * dj / (cur.g - check.g);
            conf.i       = curi;
            conf.j       = curj;
            conf.g       = curg;
            if (curg <= cur.g) {
                positions[i].push_back(conf);
                k++;
            }
            while (curg <= cur.g) {
                if (curg + 0.1 > cur.g) {
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
        if (double(k - 1) * 0.1 < sresult.pathInfo[i].sections.back().g) {
            conf.i = sresult.pathInfo[i].sections.back().i;
            conf.j = sresult.pathInfo[i].sections.back().j;
            conf.g = sresult.pathInfo[i].sections.back().g;
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

void AA_SIPP::makePrimaryPath(Node curNode)
{
    hppath.clear();
    hppath.shrink_to_fit();
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

void AA_SIPP::makeSecondaryPath(Node curNode)
{
    lppath.clear();
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

void AA_SIPP::calculateLineSegment(std::vector<Node>& line, const Node& start,
                                   const Node& goal)
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
            line.push_back(Node(i, j));
            error += delta_j;
            if ((error << 1) > delta_i) {
                j += step_j;
                error -= delta_i;
            }
        }
    } else {
        for (; j != j2; j += step_j) {
            line.push_back(Node(i, j));
            error += delta_i;
            if ((error << 1) > delta_j) {
                i += step_i;
                error -= delta_j;
            }
        }
    }
}


void debug_open(const OPEN_container& open){
    DEBUG_MSG_RED("OPEN:");
    for (auto n : open){
        DEBUG_MSG_NO_LINE_BREAK_RED(n.i);
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(n.j);
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(n.heading);
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(n.g);
        DEBUG_MSG_NO_LINE_BREAK_RED(" ");
        DEBUG_MSG_NO_LINE_BREAK_RED(n.F);
        if (n.Parent){
            DEBUG_MSG_NO_LINE_BREAK_RED(" ");
            DEBUG_MSG_NO_LINE_BREAK_RED(n.Parent->i);
            DEBUG_MSG_NO_LINE_BREAK_RED(" ");
            DEBUG_MSG_NO_LINE_BREAK_RED(n.Parent->j);
            DEBUG_MSG_RED(" ");
        }
        else{
            DEBUG_MSG_RED(" nil");
        }
    }
}
