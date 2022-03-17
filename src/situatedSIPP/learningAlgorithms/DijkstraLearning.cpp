#include "./DijkstraLearning.hpp"
#include "set"
#include "vector"


void DijkstraLearning::learn_graph(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
  std::unordered_set<RTNode>::iterator cit;
  std::multimap<double, const RTNode&> open_sorted_by_h;
  std::unordered_multiset<RTNode, boost::hash<RTNode>> close;

  for (const std::pair<int, RTNode> element: closed){
    close.insert(element.second);
  }

  for (const RTNode& closen : close){
    set_dynamic_h(closen, std::numeric_limits<double>::infinity(), false);
  }

  for (const RTNode& n: open){
    open_sorted_by_h.emplace(n.h(), n);
  }

  while (!open_sorted_by_h.empty()){
    auto oit = open_sorted_by_h.begin();
    const RTNode& n = oit->second;
    open_sorted_by_h.erase(oit);
    auto crange = close.equal_range(n);  // erase if in closed
    for (cit = crange.first; cit != crange.second; cit++){
      if ((n.Parent == cit->Parent) && (n.g() == cit->g())){
        close.erase(cit);
        break;
      }
    }
    auto prange = n.get_parents();
    for (auto parentage = prange.first; parentage != prange.second; parentage++){
      auto parent = parentage->second;
      auto orange = open_sorted_by_h.equal_range(parent.h());
      for (oit = orange.first; oit != orange.second; oit++){
        if ((oit->second == parent) && (oit->second.g() == parent.g())){
          break;
        }
      }
      double c =   n.g() - parent.g() + n.h();
      DEBUG_MSG("c");
      DEBUG_MSG(c);
      parent.debug();
      n.debug();
      if ((close.find(parent) != close.end()) && (parent.static_h() + get_dynamic_h(parent) > c)){
        set_dynamic_h(parent, c - parent.static_h());
        if (oit != orange.second){
          oit = open_sorted_by_h.erase(oit);
        }
        open_sorted_by_h.emplace(parent.h(), parent);
      }
    }
  }
}

void DijkstraLearning::learn_subintervals(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
  std::unordered_set<RTNode>::iterator cit;
  std::multimap<double, const RTNode&> open_sorted_by_h;
  std::unordered_multiset<RTNode, boost::hash<RTNode>> close;

  for (const std::pair<int, RTNode> element: closed){
    close.insert(element.second);
  }
  for (const RTNode& closen : close){
    closen.clear_dynamic_h();
    set_dynamic_h(closen, std::numeric_limits<double>::infinity(), false);
  }
  for (const RTNode& n: open){
    open_sorted_by_h.emplace(n.h(), n);
  }
  while (!open_sorted_by_h.empty()){
    auto oit = open_sorted_by_h.begin();
    const RTNode& n = oit->second;
    open_sorted_by_h.erase(oit);
    auto crange = close.equal_range(n);  // erase if in closed
    for (cit = crange.first; cit != crange.second; cit++){
      if ((n.Parent == cit->Parent) && (n.g() == cit->g())){
        close.erase(cit);
        break;
      }
    }
    n.prep_dijkstra();
    if (n.Parent != nullptr){  
      auto parent = *n.Parent;
      auto orange = open_sorted_by_h.equal_range(parent.h());
      for (oit = orange.first; oit != orange.second; oit++){
        if (oit->second == parent){
          break;
        }
      }
      double c =  n.g() - parent.g() + n.h();
      parent.add_dynamic_h(n, n.g()-parent.g(), c - parent.static_h());
      if (oit != orange.second){
        oit = open_sorted_by_h.erase(oit);
      } 
      open_sorted_by_h.emplace(parent.h(), parent);  
    }
  }
};


void DijkstraLearning::learn(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
  if (RTNode::get_dynmode() == 2){
    learn_subintervals(open,closed);
  }
  else{
    learn_graph(open,closed);
  }
  new_dynamic_h.clear();
}

