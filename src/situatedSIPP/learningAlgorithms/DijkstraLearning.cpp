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
        const RTNode * n = nullptr;
        double c = NAN;
        std::set<std::pair<double, const RTNode *>>::iterator oit;
        std::set<std::pair<double, const RTNode *>,
                 std::less<std::pair<double, const RTNode *>>> open_sorted_by_h;
        std::pair<double, const RTNode *> p;
        std::unordered_set<RTNode, boost::hash<RTNode>> close;

        for (const std::pair<int, RTNode> element: closed){
          close.insert(element.second);
        }

        // step 1
        DEBUG_MSG_RED("Closed List Contents");
        for (const RTNode& closen : close){
          closen.debug();
          RTNode copy_of_closen = RTNode(closen); 
          //copy_of_closen.set_static_h(0.0);
          copy_of_closen.set_dynamic_h(std::numeric_limits<double>::infinity());
        }
        // step 2
        DEBUG_MSG_RED("Open List Contents");
        for (const RTNode& n: open){
          n.debug();
          open_sorted_by_h.emplace(n.h(), &n);
        }
        // step 3
        while (!close.empty() && !open_sorted_by_h.empty()){// need the open check?
          DEBUG_MSG_RED("Closed List Contents");
          for (const RTNode& closen : close){
            closen.debug();
          }
          DEBUG_MSG_RED("Open List Contents");
          for (const std::pair<double, const RTNode *>& element: open_sorted_by_h){
            element.second->debug();
          }
          oit = open_sorted_by_h.begin();
          n = oit->second;
          open_sorted_by_h.erase(oit);
          cit = close.find(*n);  // erase if in closed
          if (cit != close.end()){
            close.erase(cit);
          }
          auto prange = n->get_parents();
          for (auto parent = prange.first; parent != prange.second; parent++){
            cit = close.find(parent->second);
            if (cit != close.end()){
             c =  cost(*n, parent->second) + n->h();
             if (parent->second.h() > c){
               p = std::pair<double, RTNode *>(parent->second.h(), &(parent->second)); // parent prior to updating h
               parent->second.set_dynamic_h(c - parent->second.static_h());  // update h in record
               oit = open_sorted_by_h.find(p);
               if(oit == open_sorted_by_h.end()){
                 open_sorted_by_h.emplace(parent->second.h(), &(parent->second));
               }
               else{
                 open_sorted_by_h.erase(oit);
                 open_sorted_by_h.emplace(parent->second.h(), &(parent->second));
               }
             }
            }
          }
        }
        //closed.clear();
    };
