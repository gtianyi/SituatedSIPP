#include "./learningAlgorithmBase.hpp"
#include "set"
#include "vector"

// needs to know about agent speed
auto LearningAlgorithm::cost(const Node& n1, const Node& n2) -> double{
  return sqrt(pow(n1.i - n2.i, 2) + pow(n1.j - n2.j, 2))/curagent_speed;
}

auto LearningAlgorithm::get_h(const Node& n) const -> double{
      auto it = learned_h.find(n);
      if (it == learned_h.end()){
        return n.F - n.g;
      }
      return it->second;
    }

void LearningAlgorithm::set_h(const Node& n, double h){
      learned_h[n] = h;
    }

void LearningAlgorithm::setAgentSpeed(double speed){
  curagent_speed = speed;
}

void LearningAlgorithm::debug_node(const Node& n){
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

void LearningAlgorithm::update_nodes(OPEN_container& open, std::unordered_multimap<int, Node>& closed){
  Node n;
  //int key;
  std::vector<std::pair<int, Node>> closed_holder;
  closed_holder.resize(closed.size());
  for (auto it = open.get<0>().begin(); it != open.get<0>().end(); it++) {
    n = *it;
    n.F = n.g + get_h(n);
    open.replace(it, n);
  }
  /*
  for (auto it = closed.begin(); it != closed.end(); it++) {
    key = it->first;
    n = it->second;
    n.F = n.g + get_h(n);
    closed_holder.emplace_back(key,n);
  }
  closed.clear();
  for (auto it = closed_holder.begin(); it != closed_holder.end(); it++) {
    key = it->first;
    n = it->second;
    closed.emplace(key, n);
  }
  */
}
