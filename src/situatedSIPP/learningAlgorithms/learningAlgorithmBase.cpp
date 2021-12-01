#include "./learningAlgorithmBase.hpp"
#include "set"
#include "vector"

// needs to know about agent speed
auto LearningAlgorithm::cost(const RTNode& n1, const RTNode& n2) -> double{
  DEBUG_MSG_RED("Speed");
  DEBUG_MSG_RED(curagent_speed);
  return sqrt(pow(n1.i - n2.i, 2) + pow(n1.j - n2.j, 2))/curagent_speed;
}

void LearningAlgorithm::setAgentSpeed(double speed){
  curagent_speed = speed;
}

