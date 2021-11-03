#pragma once
#include <unordered_set>

#include "../../structs.h"

class LearningAlgorithm
{
public:
    virtual void learn(OPEN_container&                     open,
                       std::unordered_set<Node>& close) = 0;
};
