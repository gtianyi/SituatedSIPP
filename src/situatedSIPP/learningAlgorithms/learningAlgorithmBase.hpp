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
  virtual void learn(RTOPEN_container& open,
                       std::unordered_multimap<int, RTNode>& close) = 0;
};
