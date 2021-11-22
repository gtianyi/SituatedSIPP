#include "./learningAlgorithmBase.hpp"
#include "set"
#include "vector"

// needs to know about agent speed
auto LearningAlgorithm::cost(const RTNode& n1, const RTNode& n2) -> double{
  return sqrt(pow(n1.i - n2.i, 2) + pow(n1.j - n2.j, 2))/curagent_speed;
}

auto LearningAlgorithm::get_h(const RTNode& n) const -> double{
      auto it = learned_h.find(n);
      if (it == learned_h.end()){
        return n.F() - n.g();
      }
      return it->second;
    }

void LearningAlgorithm::set_h(const RTNode& n, double h){
      learned_h[n] = h;
    }

void LearningAlgorithm::setAgentSpeed(double speed){
  curagent_speed = speed;
}

void LearningAlgorithm::debug_node(const RTNode& n){
  DEBUG_MSG_NO_LINE_BREAK_RED(n.i);
  DEBUG_MSG_NO_LINE_BREAK_RED(" ");
  DEBUG_MSG_NO_LINE_BREAK_RED(n.j);
  DEBUG_MSG_NO_LINE_BREAK_RED(" ");
  DEBUG_MSG_NO_LINE_BREAK_RED(n.interval.begin);
  DEBUG_MSG_NO_LINE_BREAK_RED(" ");
  DEBUG_MSG_NO_LINE_BREAK_RED(n.interval.end);
  DEBUG_MSG_NO_LINE_BREAK_RED(" ");
  DEBUG_MSG_RED(get_h(n));
}
