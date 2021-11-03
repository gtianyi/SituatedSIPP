#pragma once
#include "learningAlgorithmBase.hpp"

class NoLearning : public LearningAlgorithm
{
public:
    virtual void learn(OPEN_container&  open,
                       std::unordered_set<Node>& close) override
    {
        open = open;
        close.clear();
    }
};
