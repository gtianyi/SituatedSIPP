#pragma once
#include "cmath"
#include "../../debug.h"
#include "unordered_set"
#include "unordered_map"
#include "learningAlgorithmBase.hpp"

class DijkstraLearning : public LearningAlgorithm{
private:
    std::unordered_map<RTNode, double, boost::hash<RTNode>> new_dynamic_h;
    void learn_graph(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& close);
    void learn_subintervals(RTOPEN_container& open, std::unordered_multimap<int, RTNode>& close);
public:
    void learn(RTOPEN_container& open,
                       std::unordered_multimap<int, RTNode>& close) override;
   
    double get_dynamic_h(const RTNode& n) const{
        auto loc = new_dynamic_h.find(n);
        //DEBUG_MSG(loc);
        if (loc == new_dynamic_h.end()){
            return n.dynamic_h();
        }
        return loc->second;
    }

    void set_dynamic_h(const RTNode& n, double value, bool commit = true){
        auto loc = new_dynamic_h.find(n);
        if (loc != new_dynamic_h.end()){
            new_dynamic_h.erase(loc);
        }
        new_dynamic_h.emplace(n, value);
        loc = new_dynamic_h.find(n);
        if (commit){
            n.set_dynamic_h(value);
        }
    }

    void dump_ndh(){
        DEBUG_MSG("DUMPING NDH");
        for (auto it: new_dynamic_h){
            it.first.debug();
            DEBUG_MSG(it.second);
            DEBUG_MSG("");
        }
        RTNode::dump_dh();
    }
};