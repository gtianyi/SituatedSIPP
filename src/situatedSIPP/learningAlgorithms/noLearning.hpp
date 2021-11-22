#pragma once
#include "learningAlgorithmBase.hpp"

class NoLearning : public LearningAlgorithm{
public:
    virtual void learn(RTOPEN_container&  /*open*/,
                       std::unordered_multimap<int, RTNode>& close) override
    {
        close.clear();
    }
};
