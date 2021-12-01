#pragma once
#include "cmath"
#include "../../debug.h"
#include "unordered_set"
#include "unordered_map"
#include "learningAlgorithmBase.hpp"

class PlrtaLearning : public LearningAlgorithm{
public:
    void learn(RTOPEN_container& open,
                       std::unordered_multimap<int, RTNode>& close) override;
};