#pragma once
#include "../../debug.h"
#include "learningAlgorithmBase.hpp"

class DijkstraLearning : public LearningAlgorithm
{
public:
    DijkstraLearning(){}

    virtual void learn(OPEN_container&,
                       std::unordered_multimap<int, Node>& close) override
    {

        DEBUG_MSG_RED("Devin use this DEBUG COLOR");
        //DEBUG_MSG_NO_LINE_BREAK_RED("Devin use this DEBUG COLOR, noline break");

        // Devin TODO
        // using LSSLRTA* style Dijkstra learning
        // 1. mark everthing in closed list as infinit
        // 2. Order open by h
        // 3. Perform reverse dijkstra while closed is not empy. Start re-ordered top node in the open list, backup its h value to the
        // local search space, by tracing the parent pointer
        // reference code https://github.com/gtianyi/rationalRealtimeSearch/blob/master/cpp/learningAlgorithms/Dijkstra.h
        close.clear();
    }
};
