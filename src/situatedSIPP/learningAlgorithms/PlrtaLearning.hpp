#pragma once
#include "cmath"
#include "../../debug.h"
#include "unordered_set"
#include "unordered_map"
#include "learningAlgorithmBase.hpp"

class PlrtaLearning : public LearningAlgorithm{    
private:
    std::unordered_map<std::pair<int, int>, double, boost::hash<std::pair<int, int>>> new_static_h;
    std::unordered_map<RTNode, double, boost::hash<RTNode>> new_dynamic_h;
public:
    void learn(RTOPEN_container& open,
                       std::unordered_multimap<int, RTNode>& close) override;
    double get_static_h(const RTNode& n) const{
        auto key = std::pair<int, int>(n.i, n.j); 
        auto loc = new_static_h.find(key);
        if (loc == new_static_h.end()){
            return n.static_h();
        }
        return loc->second;
    }

    void set_static_h(int i, int j, double value, bool commit = true){
        auto key = std::pair<int, int>(i, j); 
        new_static_h.emplace(key, value);
        if (commit){
            RTNode n = RTNode(i, j);
            n.set_static_h(value);
        }
    }

    double get_dynamic_h(const RTNode& n) const{
        auto loc = new_dynamic_h.find(n);
        if (loc == new_dynamic_h.end()){
            return n.dynamic_h();
        }
        return loc->second;
    }

    void set_dynamic_h(const RTNode& n, double value, bool commit = true){
        new_dynamic_h.emplace(n, value);
        if (commit){
            n.set_dynamic_h(value);
        }
    }
};