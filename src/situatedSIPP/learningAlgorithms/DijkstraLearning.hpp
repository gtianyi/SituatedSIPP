#pragma once
#include "cmath"
#include "../../debug.h"
#include "unordered_set"
#include "unordered_map"
#include "learningAlgorithmBase.hpp"

auto cost(const Node& n1, const Node& n2) -> double;

class DijkstraLearning : public LearningAlgorithm
{
private:
    std::unordered_map<Node, double, boost::hash<Node>> learned_h;
public:
    auto get_h(const Node& n) const -> double;
    void set_h(const Node& n, double h);
    virtual void learn(OPEN_container& open,
                       std::unordered_set<Node>& close) override;
};
auto lt(Node const &n1, Node const &n2, DijkstraLearning const &dl) -> bool;
