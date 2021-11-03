#pragma once
#include "cmath"
#include "../../debug.h"
#include "learningAlgorithmBase.hpp"

double cost(Node * n1, Node * n2){
  return sqrt(pow(n1->i - n2->i, 2) + pow(n1->j - n2->j, 2));
}

class DijkstraLearning : public LearningAlgorithm
{
private:
  std::unordered_map<Node, double> learned_h;
public:
    double get_h(Node * n){
      std::unordered_multimap<State, double>::iterator it = learned_h.find(n));
      if (it == learned_h.end()){
        return n->F - n->g;
      }
      else{
        return it->second;
      }
    }

    void set_h(Node n, double h){
      learned_h[n] = h;
    }

    virtual void learn(OPEN_container& open,
                       std::unordered_set<Node>& close) override
    {

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
        priority_queue <Node, vector<Node>,
         [](Node const &n1, Node const &n2) {
            return n1.h < n2.h;} >
        open_sorted_by_h;
        std::unordered_set<Node>::iterator cit;
        double acc;
        State s;
        Node n;
        double duration;
        // step 1
        for (std::pair<int, Node> element : close){
          set_h(element.second, std::numeric_limits<double>::infinity());
        }
        // step 2
        for (Node n: open){
          open_sorted_by_h.emplace(n);
        }
        // step 3
        while (!closed.empty()){// need the open check?
          n = open_sorted_by_h.top();
          open_sorted_by_h.pop();
          cit = close.find(n);  // erase if in closed
          if (cit != close.end()){
            close.erase(cit);
          }
          if (n.parent){
            cit = close.find(n.parent)
            if ((cit != close.end()) && (get_h(n.parent) > (cost(&n, n.parent) + get_h(&n)))){
              
            }
          }
          duration = cost()
          acc = get_h(n);
          if n in
        }

        close.clear();
    }
};
