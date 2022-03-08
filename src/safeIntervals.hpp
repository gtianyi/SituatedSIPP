#pragma once

#include "mission.h"
#include "structs.h"
#include <cstddef>

class SafeIntervals{
    private:
        std::vector<std::set<std::pair<double, double>>> _safe_intervals;
        const Map& map;
        inline int get_index(int i, int j) const{return i * (int)map.width + j;}
    public:
        SafeIntervals(const Mission& mission); // populates safe intervals
        bool isSafe(size_t i, size_t j, double t);
        std::set<std::pair<double, double>> get_safe_intervals(size_t i, size_t j); // return safe intervals at location.
        std::pair<std::set<std::pair<double, double>>::iterator, std::set<std::pair<double, double>>::iterator> get_safe_intervals(size_t i, size_t j, double beginning); // now get rid of any that end before beginning.
        std::pair<std::set<std::pair<double, double>>::iterator, std::set<std::pair<double, double>>::iterator> get_safe_intervals(size_t i, size_t j, double beginning, double ending); // and start after ending. 
};

std::pair<double, double> collision_interval(int i, int j, const Node& source, const Node& destination);