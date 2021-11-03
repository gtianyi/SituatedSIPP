#pragma once
#include <unordered_map>

#include "../../structs.h"

class LearningAlgorithm
{
public:
    virtual void learn(OPEN_container&                     open,
                       std::unordered_multimap<int, Node>& close) = 0;
};
