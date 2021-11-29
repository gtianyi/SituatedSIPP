#include "./DijkstraLearning.hpp"
#include "set"
#include "vector"

auto lt(RTNode const &n1, RTNode const &n2, DijkstraLearning const &dl) -> bool {
   return dl.get_h(n1) > dl.get_h(n2);
 }

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
          debug_node(closen);
          set_h(closen, std::numeric_limits<double>::infinity());
        }
        // step 2
        DEBUG_MSG_RED("Open List Contents");
        for (const RTNode& n: open){
          debug_node(n);
          open_sorted_by_h.emplace(get_h(n), &n);
        }
        // step 3
        while (!close.empty() && !open_sorted_by_h.empty()){// need the open check?
          DEBUG_MSG_NO_LINE_BREAK_RED("Iteration: ");
          DEBUG_MSG_RED(iteracc++);
          DEBUG_MSG_RED("Closed List Contents");
          for (const RTNode& closen : close){
            debug_node(closen);
          }
          DEBUG_MSG_RED("Open List Contents");
          for (const std::pair<double, const RTNode *>& element: open_sorted_by_h){
            debug_node(*element.second);
          }
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
               p = std::pair<double, RTNode *>(get_h(*(n->Parent)), n->Parent); // parent prior to updating h
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
        //closed.clear();
    };
