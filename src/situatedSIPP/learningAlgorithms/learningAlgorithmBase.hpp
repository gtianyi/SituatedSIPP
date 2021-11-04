#pragma once
#include <unordered_map>
#include "../../debug.h"
#include "../../structs.h"

class LearningAlgorithm
{
private:
  std::unordered_map<Node, double, boost::hash<Node>> learned_h;
  double curagent_speed;
public:
  void setAgentSpeed(double speed);
  auto cost(const Node& n1, const Node& n2) -> double;
  void debug_node(const Node& n);
  auto get_h(const Node& n) const -> double;
  void set_h(const Node& n, double h);
  virtual void learn(OPEN_container&                     open,
                       std::unordered_multimap<int, Node>& close) = 0;
};
