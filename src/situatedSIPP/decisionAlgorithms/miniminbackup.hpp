#pragma once
#include "decisionAlgorithmBase.hpp"

template<typename SearchClass>
class MiniminBackup : public DecisionAlgorithm<SearchClass>
{
public:
    RTNode backupAndRecordPartialPlan(
      const RTNode& curNode, const timeval& begin, const timeval& end,
      const int goal_i, const int goal_j, std::vector<RTNode>& hppath,
      std::list<RTNode>& lppath, SearchClass* searchClassPtr) override
    {

        // Tianyi note: this goal test might be too much simplified, check
        // AA_SIPP::stpCriterion
        DEBUG_MSG_RED("Pre findmin");
        auto bestFrontierNode = searchClassPtr->findMin();
        
        DEBUG_MSG("goal i, j: " << goal_i << " " << goal_j);

        if (curNode.i == goal_i && curNode.j == goal_j) {
            bestFrontierNode = curNode;
        }

        auto cur       = bestFrontierNode;
        auto parentPtr = bestFrontierNode.Parent;

        while (parentPtr != nullptr && parentPtr->Parent != nullptr) {
            cur       = *parentPtr;
            parentPtr = cur.Parent;
        }

        /*DEBUG_MSG("curNode after search i, j, g: "*/
                  //<< curNode.i << " " << curNode.j << " " << curNode.g);
        //DEBUG_MSG("best frontier i, j, g: " << bestFrontierNode.i << " "
                                            //<< bestFrontierNode.j << " "
                                            //<< bestFrontierNode.g);
        //DEBUG_MSG("best TLA after search i, j, g: " << cur.i << " " << cur.j
                                                    //<< " " << cur.g);

        searchClassPtr->recordToOnlinePath(*parentPtr, bestFrontierNode, begin, end);
        hppath.push_back(cur);
        lppath.push_back(cur);

        return cur;
    }
};
