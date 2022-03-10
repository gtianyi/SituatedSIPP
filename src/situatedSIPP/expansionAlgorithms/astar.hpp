#pragma once
#include "expansionAlgorithmBase.hpp"

template<typename SearchClass>
class Astar : public ExpansionAlgorithm<SearchClass>
{
public:
    void runSearch(RTNode& curNode, RTNode& goalNode, const Map& map,
                   std::unordered_multimap<int, RTNode>& close, int& reexpanded,
                   std::list<RTNode>& reexpanded_list,
                   SearchClass*       searchClassPtr, SafeIntervals& safe_intervals) override
    {

        int curExpansion(0);
        // expansion phase
        while (!searchClassPtr->stopCriterion(curNode, goalNode) && curExpansion < searchClassPtr->lookaheadBudget) {
            curExpansion++;
            curNode = searchClassPtr->findMin();
            auto range = close.equal_range(curNode.i * map.width + curNode.j);
            for (auto it = range.first; it != range.second; it++) {
                if (it->second.interval_id ==
                    curNode.interval_id) // && it->second.heading_id ==
                                         // curNode.heading_id)
                {
                    reexpanded++;
                    reexpanded_list.push_back(curNode);
                    if (searchClassPtr->config->planforturns &&
                        it->second.heading_id != curNode.heading_id) {
                        reexpanded--;
                    }
                    break;
                }
            }
            // curNode.close_id = close_id;
            // close_id++;
            close.insert({curNode.i * map.width + curNode.j, curNode});
            for (RTNode s : searchClassPtr->findSuccessors(curNode, map, safe_intervals)){
                if (searchClassPtr->config->use_likhachev) {
                    range    = close.equal_range(s.i * map.width + s.j);
                    bool add = true;
                    for (auto it = range.first; it != range.second; it++) {
                        if (it->second.interval_id == curNode.interval_id &&
                            (!searchClassPtr->config->planforturns ||
                             it->second.heading_id == curNode.heading_id)) {
                            add = false;
                            break;
                        }
                    }
                    s.optimal = false;
                    if (add) {
                        searchClassPtr->addOpen(s);
                    }
                    if (curNode.optimal) {
                        s.optimal = true;
                        //s.set_static_h(searchClassPtr->config->h_weight * (s.g() + searchClassPtr->getHValue(s.i, s.j)));
                        searchClassPtr->addOpen(s);
                    }
                } else {
                    
                    searchClassPtr->addOpen(s);
                    //DEBUG_MSG("    open size " << open.size());
                }
            }
        }
        searchClassPtr->sresult.expansions += curExpansion;
    }
};
