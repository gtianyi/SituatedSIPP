#pragma once
#include <unordered_map>
#include "../../debug.h"
#include "../structs.h"

class LearningAlgorithm
{
private:
  std::unordered_map<RTNode, double, boost::hash<RTNode>> learned_h;
  double curagent_speed;
public:
  void setAgentSpeed(double speed);
  auto cost(const RTNode& n1, const RTNode& n2) -> double;
  void debug_node(const RTNode& n);
  auto get_h(const RTNode& n) const -> double;
  void set_h(const RTNode& n, double h);
  virtual void learn(RTOPEN_container& open,
                       std::unordered_multimap<int, RTNode>& close) = 0;
};
