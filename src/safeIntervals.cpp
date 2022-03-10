#include "safeIntervals.hpp"
#include "debug.h"
#include "dynamicobstacles.h"
#include "structs.h"
#include <cmath>
#include <cstddef>


bool SafeIntervals::isSafe(int i, int j, double t){
    auto loc = get_safe_intervals(i, j, t); // first safe interval starting after t
    return (loc.begin() != loc.end() && loc.begin()->first <= t && loc.begin()->second >= t); // otherwise safe if in the one earlier.
}

boost::container::flat_set<std::pair<double, double>> SafeIntervals::get_safe_intervals(int i, int j){
    int ind = get_index(i, j);
    if (!_generated_safe_intervals[ind]){
        _generated_safe_intervals[ind] = true;
        lazy_compute_safe_intervals(i, j,obstacles, map, agent_size, _safe_intervals[ind]);
    }
    return *_safe_intervals[ind];
}

boost::container::flat_set<std::pair<double, double>> SafeIntervals::get_safe_intervals(int i, int j, double beginning){
    auto key = std::pair<double, double>(beginning, beginning);
    auto si = get_safe_intervals(i, j);
    auto loc = si.lower_bound(key); 
    if (loc != si.begin()){ //nothing earlier means unsafe
            --loc;
    }
    return boost::container::flat_set<std::pair<double, double>>(loc, si.end());
}

SafeInterval SafeIntervals::getSafeInterval(int i, int j, double beginning){
    auto safe_intervals = get_safe_intervals(i, j, beginning);
    if (safe_intervals.begin() == safe_intervals.end()|| beginning < safe_intervals.begin()->first){
        return SafeInterval(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
    }
    return SafeInterval(safe_intervals.begin()->first, safe_intervals.begin()->second);
}

boost::container::flat_set<std::pair<double, double>> SafeIntervals::get_safe_intervals(int i, int j, double beginning, double ending){
    auto key = std::pair<double, double>(ending, ending);
    auto si = get_safe_intervals(i, j);
    auto loc = si.equal_range(key);
    if (loc.first != si.begin()){ //nothing earlier means unsafe
            --loc.first;
    }
    auto retval = get_safe_intervals(i, j, beginning); 
    return boost::container::flat_set<std::pair<double, double>>(loc.first, loc.second);
}


SafeIntervals::SafeIntervals(const Map& map, const DynamicObstacles& obstacles, double agent_size) : map(map),obstacles(obstacles), agent_size(agent_size){
    // precompute safe intervals
    size_t size = (long)map.height*(long)map.width;
    _safe_intervals.reserve(size);
    _generated_safe_intervals.reserve(size);
    for (int i=0; i<map.height; i++){
        for (int j=0; j < map.width; j++){
            _safe_intervals.emplace_back(new boost::container::flat_set<std::pair<double, double>>());
            _generated_safe_intervals.emplace_back(false);
        }
    }
}


void SafeIntervals::dump(const std::string& fn) const {
    (void)fn;
    /*
    for (int i=0; i<map.height; i++){
        for (int j=0; j < map.width; j++){
            int interval_key = get_index(i, j);

        }
    }
    */
}
