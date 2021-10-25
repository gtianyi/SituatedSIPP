#pragma once
#include "learningAlgorithmBase.hpp"

class NoLearning : public LearningAlgorithm
{
public:
    virtual void learn(OPEN_container&,
                       std::unordered_multimap<int, Node>& close) override
    {
        close.clear();
    }
};
