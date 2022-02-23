#include "./PlrtaLearning.hpp"
#include "set"
#include "vector"

void PlrtaLearning::learn_graph(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
  std::unordered_multiset<RTNode>::iterator cit;
  std::multimap<double, const RTNode&> open_sorted_by_h;
  std::unordered_multiset<RTNode, boost::hash<RTNode>> close;

  for (const std::pair<int, RTNode> element: closed){
    close.insert(element.second);
  }

  for (const RTNode& closen : close){
    set_static_h(closen.i, closen.j, std::numeric_limits<double>::infinity(), false);
    if (RTNode::get_dynmode() == 2){
      closen.clear_dynamic_h();
    }
    set_dynamic_h(closen, std::numeric_limits<double>::infinity(), false);
  }
  for (const RTNode& n: open){
    open_sorted_by_h.emplace(n.h(), n);
  }
  while (!open_sorted_by_h.empty()){// need the open check?
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
      bool changed = false;
      double c =  cost(n, parent) + n.static_h();
      if (get_static_h(parent) > c){
        set_static_h(parent.i, parent.j, c); 
        changed = true;
      }
      c = n.dynamic_g() + n.dynamic_h() - parent.dynamic_g();
      if ((c < get_dynamic_h(parent))){
        set_dynamic_h(parent, c);
        changed = true;
      }
      if (changed){
        if(oit != open_sorted_by_h.end()){
          open_sorted_by_h.erase(oit);
        }
        open_sorted_by_h.emplace(parent.h(), parent);
      }
    }
  }
}

void PlrtaLearning::learn_subintervals(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
  std::unordered_multiset<RTNode>::iterator cit;
  std::multimap<double, const RTNode&> open_sorted_by_h;
  std::unordered_multiset<RTNode, boost::hash<RTNode>> close;

  for (const std::pair<int, RTNode> element: closed){
    close.insert(element.second);
  }
  for (const RTNode& closen : close){
    //closen.debug();
    set_static_h(closen.i, closen.j, std::numeric_limits<double>::infinity(), false);
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
      auto parent = *(n.Parent);
      auto orange = open_sorted_by_h.equal_range(parent.h());
      for (oit = orange.first; oit != orange.second; oit++){
        if (oit->second == parent){
          break;
        }
      }
      double c =  cost(n, parent) + n.static_h();
      if (get_static_h(parent) > c){
        set_static_h(parent.i, parent.j, c);
      }
      c = n.dynamic_g() + n.dynamic_h() - parent.dynamic_g();
      if (true || (c < get_dynamic_h(parent))){
        parent.add_dynamic_h(n, cost(n, parent), c);
        if(oit != open_sorted_by_h.end()){
          open_sorted_by_h.erase(oit);
        }
        open_sorted_by_h.emplace(parent.h(), parent);
      }
    }
  }
};

void PlrtaLearning::learn(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
  if (RTNode::get_dynmode() == 2){
    learn_subintervals(open,closed);
  }
  else{
    learn_graph(open,closed);
  }
  new_static_h.clear();
  new_dynamic_h.clear();
}