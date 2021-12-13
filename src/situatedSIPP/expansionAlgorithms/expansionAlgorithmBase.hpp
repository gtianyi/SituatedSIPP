#pragma once
#include <unordered_map>
#include <vector>

#include "../../structs.h"
#include "../../map.h"
#include "../structs.h"

template<typename SearchClass>
class ExpansionAlgorithm
{
public:
    virtual void runSearch(RTNode& curNode, RTNode& goalNode, const Map& map, std::unordered_multimap<int, RTNode>& close, int& reexpanded, std::list<RTNode>& reexpanded_list, SearchClass* searchClassPtr) = 0;
};
