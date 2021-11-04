#pragma once
#include "cmath"
#include "../../debug.h"
#include "unordered_set"
#include "unordered_map"
#include "learningAlgorithmBase.hpp"

class DijkstraLearning : public LearningAlgorithm
{
public:
    void learn(OPEN_container& open,
                       std::unordered_multimap<int, Node>& close) override;
};
auto lt(Node const &n1, Node const &n2, DijkstraLearning const &dl) -> bool;
