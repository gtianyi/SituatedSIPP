#include "./PlrtaLearning.hpp"
#include "set"
#include "vector"

void PlrtaLearning::learn(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
        // Devin TODO
        // using LSSLRTA* style Dijkstra learning
        // 1. mark everthing in closed list as infinity
        // 2. Order open by h
        // 3. Perform reverse dijkstra while closed is not empy. Start re-ordered top node in the open list, backup its h value to the
        // local search space, by tracing the parent pointer
        // reference code https://github.com/gtianyi/rationalRealtimeSearch/blob/master/cpp/learningAlgorithms/Dijkstra.h
        // keep track of heuristic values somewhere.
        std::unordered_set<RTNode>::iterator cit;
        const RTNode * n = nullptr;
        double c = NAN;
        std::set<std::pair<double, const RTNode *>>::iterator oit;
        std::set<std::pair<double, const RTNode *>,std::less<std::pair<double, const RTNode *>>> open_sorted_by_h;
        std::pair<double, const RTNode *> p;
        std::unordered_set<RTNode, boost::hash<RTNode>> close;
        for (const std::pair<int, RTNode> element: closed){
          close.insert(element.second);
        }


        // step 1
        //DEBUG_MSG("CLOSED");
        for (const RTNode& closen : close){
            //closen.debug();
            set_static_h(closen.i, closen.j, std::numeric_limits<double>::infinity(), false);
            set_dynamic_h(closen, std::numeric_limits<double>::infinity(), false);
        }
        // step 2
        //DEBUG_MSG("OPEN");
        for (const RTNode& n: open){
          //n.debug();
          open_sorted_by_h.emplace(n.h(), &n);
        }
        // step 3
        while (!open_sorted_by_h.empty()){// need the open check?
          oit = open_sorted_by_h.begin();
          n = oit->second;
          open_sorted_by_h.erase(oit);
          cit = close.find(*n);  // erase if in closed
          if (cit != close.end()){
            close.erase(cit);
          }
          auto prange = n->get_parents();
          for (auto parentage = prange.first; parentage != prange.second; parentage++){
            auto parent = parentage->second;
            //cit = close.find(parentage->second); 
            //cit->debug();
            bool changed = false;
            c =  cost(*n, parent) + n->static_h();
            /*
            DEBUG_MSG("calculation:");
            DEBUG_MSG(c);
            DEBUG_MSG(get_static_h(parent));
            */
            if (get_static_h(parent) > c){
              set_static_h(parent.i, parent.j, c);  // update h in record
              //DEBUG_MSG(parent.static_h());
              //DEBUG_MSG(" ");

              changed = true;
              }
            c = n->dynamic_g() + n->dynamic_h() - parent.dynamic_g();
            if (c < get_dynamic_h(parent)){
                set_dynamic_h(parent, c);
                changed = true;
              }
            if (changed){
              p = std::pair<double, RTNode *>(parent.h(), &parent); // parent prior to updating h
              oit = open_sorted_by_h.find(p);
              if(oit == open_sorted_by_h.end()){
                open_sorted_by_h.emplace(parent.h(), &parent);
                }
              else{
                open_sorted_by_h.erase(oit);
                open_sorted_by_h.emplace(parent.h(), &parent);
                }
              }
              
          }
        }
        new_static_h.clear();
        new_dynamic_h.clear();
        //closed.clear();
    };
