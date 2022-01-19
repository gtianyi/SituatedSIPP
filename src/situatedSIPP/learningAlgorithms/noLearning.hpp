#pragma once
#include <tuple>
#include "learningAlgorithmBase.hpp"

class NoLearning : public LearningAlgorithm{
public:
    virtual void learn(RTOPEN_container&  /*open*/,
                       std::unordered_multimap<int, RTNode>& close) override
    {
            std::ignore = close;
    }
};
