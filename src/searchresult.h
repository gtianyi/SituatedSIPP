#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include <string>
#include <vector>
#include <list>

#include "structs.h"

struct ResultPathInfo
{
    bool pathfound;
    double pathlength;
    double runtime;
    std::list<Node> path;
    std::vector<ResultPathInfo> iterationPath;
    std::vector<Node> sections;
    int expanded;
    int generated;
    int reopened;
    int reexpanded;
    std::list<Node> reexpanded_list;

    ResultPathInfo()
    {
        runtime = 0;
        pathfound = false;
        pathlength = 0;
        path.clear();
        sections.clear();
    }
};

struct SearchResult
{
    bool pathfound;
    std::string agentFate; // survived, died, trapped, timed out
    double makespan;
    double flowtime;
    double runtime;
    unsigned int agents;
    int agentsSolved;
    int tries;
    unsigned long expansions;
    std::string timingInformation;
    std::vector<ResultPathInfo> pathInfo;


    SearchResult() : pathInfo(1)
    {
        pathfound = false;
        agentFate = "survived";
        runtime = 0;
        flowtime = 0;
        makespan = 0;
        agents = 0;
        expansions = 0;
        timingInformation = "";
    }

    ~SearchResult()
    {
        pathInfo.clear();
    }

};

#endif // SEARCHRESULT_H
