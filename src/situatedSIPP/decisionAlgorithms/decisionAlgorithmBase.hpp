#pragma once
#include <unordered_map>
#include <vector>

#include "../../structs.h"
#include "../structs.h"

template<typename SearchClass>
class DecisionAlgorithm
{
public:
    virtual RTNode backupAndRecordPartialPlan(
      const RTNode& curNode, const timeval& begin, const timeval& end,
      const int goal_i, const int goal_j, std::vector<RTNode>& hppath,
      std::list<RTNode>& lppath, SearchClass* searchClassPtr) = 0;
};
