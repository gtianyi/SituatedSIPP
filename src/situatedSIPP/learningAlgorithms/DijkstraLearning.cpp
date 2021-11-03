#pragma once
#include "./DijkstraLearning.hpp"
#include "cmath"
#include "../../debug.h"
#include "unordered_set"
#include "set"
#include "unordered_map"
#include "learningAlgorithmBase.hpp"
#include "vector"


auto cost(const Node& n1, const Node& n2) -> double{
  return sqrt(pow(n1.i - n2.i, 2) + pow(n1.j - n2.j, 2));
}

auto lt(Node const &n1, Node const &n2, DijkstraLearning const &dl) -> bool {
   return dl.get_h(n1) < dl.get_h(n2);
 }


auto DijkstraLearning::get_h(const Node& n) const -> double{
      auto it = learned_h.find(n);
      if (it == learned_h.end()){
        return n.F - n.g;
      }
      return it->second;
    }

void DijkstraLearning::set_h(const Node& n, double h){
      learned_h[n] = h;
    }



void DijkstraLearning::learn(OPEN_container& open, std::unordered_set<Node>& close){

        DEBUG_MSG_RED("Devin use this DEBUG COLOR");
        //DEBUG_MSG_NO_LINE_BREAK_RED("Devin use this DEBUG COLOR, noline break");

        // Devin TODO
        // using LSSLRTA* style Dijkstra learning
        // 1. mark everthing in closed list as infinity
        // 2. Order open by h
        // 3. Perform reverse dijkstra while closed is not empy. Start re-ordered top node in the open list, backup its h value to the
        // local search space, by tracing the parent pointer
        // reference code https://github.com/gtianyi/rationalRealtimeSearch/blob/master/cpp/learningAlgorithms/Dijkstra.h
        // keep track of heuristic values somewhere.
        std::unordered_set<Node>::iterator cit;
        double acc = NAN;
        Node * n = nullptr;
        double c = NAN;
        std::set<std::pair<double, Node *>>::iterator oit;
        std::set<std::pair<double, Node *>,
                 std::greater<std::pair<double, Node *>>> open_sorted_by_h;
        std::pair<double, Node *> p;
        // step 1
        for (const Node& closen : close){
          set_h(closen, std::numeric_limits<double>::infinity());
        }
        // step 2
        for (const Node& n: open){
          open_sorted_by_h.emplace(get_h(n), &n);
        }
        // step 3
        while (!close.empty() && !open_sorted_by_h.empty()){// need the open check?
          oit = open_sorted_by_h.begin();
          n = oit->second;
          open_sorted_by_h.erase(oit);
          cit = close.find(*n);  // erase if in closed
          if (cit != close.end()){
            close.erase(cit);
          }
          if (n->Parent != nullptr){
            cit = close.find(*(n->Parent));
            if (cit != close.end()){
             c =  cost(*n, *(n->Parent)) + get_h(*n);
             if (get_h(*(n->Parent)) > c){
               p = std::pair<double, Node *>(get_h(*(n->Parent)), n->Parent); // parent prior to updating h
               set_h(*(n->Parent), c);  // update h in record
               oit = open_sorted_by_h.find(p);
               if(oit == open_sorted_by_h.end()){
                 open_sorted_by_h.emplace(get_h(*(n->Parent)), n->Parent);
               }
               else{
                 open_sorted_by_h.erase(oit);
                 open_sorted_by_h.emplace(get_h(*(n->Parent)), n->Parent);
               }
             }
            }
          }
        }
          //duration = cost()
          //acc = get_h(n);
          //if n in
        }

        close.clear();
    }
};
