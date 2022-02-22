#include "./DijkstraLearning.hpp"
#include "set"
#include "vector"

void DijkstraLearning::learn(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& closed){
        // Devin TODO
        // using LSSLRTA* style Dijkstra learning
        // 1. mark everthing in closed list as infinity
        // 2. Order open by h
        // 3. Perform reverse dijkstra while closed is not empy. Start re-ordered top node in the open list, backup its h value to the
        // local search space, by tracing the parent pointer
        // reference code https://github.com/gtianyi/rationalRealtimeSearch/blob/master/cpp/learningAlgorithms/Dijkstra.h
        // keep track of heuristic values somewhere.
        std::unordered_set<RTNode>::iterator cit;
        double c = NAN;
        std::multimap<double, const RTNode&> open_sorted_by_h;
        std::pair<double, RTNode> p;
        std::unordered_set<RTNode, boost::hash<RTNode>> close;
        std::unordered_map<RTNode, std::unordered_set<RTNode,boost::hash<RTNode>>, boost::hash<RTNode>> touched;
        for (const std::pair<int, RTNode> element: closed){
          close.insert(element.second);
        }
        // step 1
        DEBUG_MSG("CLOSED");
        for (const RTNode& closen : close){
          if (RTNode::get_dynmode() == 2){
            closen.clear_dynamic_h();
          }
          set_dynamic_h(closen, std::numeric_limits<double>::infinity(), false);
          
        }
        // step 2
        DEBUG_MSG("OPEN");
        for (const RTNode& n: open){
          n.debug();
          open_sorted_by_h.emplace(n.h(), n);
        }
        // step 3

        while (!close.empty() && !open_sorted_by_h.empty()){// need the open check?
          auto oit = open_sorted_by_h.begin();
          const RTNode& n = oit->second;
          //DEBUG_MSG("child");
          //n.debug();
          open_sorted_by_h.erase(oit);
          cit = close.find(n);  // erase if in closed
          if (cit != close.end()){
            close.erase(cit);
          }
          //DEBUG_MSG("parents");
          if (RTNode::get_dynmode() == 2){ 
            n.prep_dijkstra();
          }
          auto prange = n.get_parents();
          for (auto parentage = prange.first; parentage != prange.second; parentage++){
            auto parent = parentage->second;
            //parent.debug();
            auto orange = open_sorted_by_h.equal_range(parent.h());
              for (oit = orange.first; oit != orange.second; oit++){
                if (oit->second == parent){
                  oit = open_sorted_by_h.erase(oit);
                  break;
                }
              }
            c =  cost(n, parent) + n.h();
            if ((close.find(parent) != close.end()) && (parent.static_h() + get_dynamic_h(parent) > c)){
              if (RTNode::get_dynmode() == 2){
                parent.add_dynamic_h(n, cost(n, parent), c - parent.static_h());
                if (touched[n].emplace(parent).second){
                  open_sorted_by_h.emplace(parent.h(), parent);  
                }

              }
              else{
                  //p = std::pair<double, RTNode>(parent.h(), parent); // parent prior to updating h
                  set_dynamic_h(parent, c - parent.static_h());
                  open_sorted_by_h.emplace(parent.h(), parent);
                }
            }
            

            /*
            DEBUG_MSG("OPEN_SORTED_BY_H:");
              for (auto n: open_sorted_by_h){
                DEBUG_MSG(n.first);
                n.second.debug();
              }
            */
          }
        }
        //dump_ndh();
        new_dynamic_h.clear();
        //closed.clear();
    };
