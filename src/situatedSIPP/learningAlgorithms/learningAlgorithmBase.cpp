#include "./learningAlgorithmBase.hpp"
#include "set"
#include "vector"
#include <cmath>

// needs to know about agent speed
double LearningAlgorithm::cost(const RTNode& n1, const RTNode& n2) const{
  return sqrt(pow(n1.i - n2.i, 2) + pow(n1.j - n2.j, 2))/curagent_speed;
}

void LearningAlgorithm::setAgentSpeed(double speed){
  curagent_speed = speed;
}

