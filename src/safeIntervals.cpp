#include "safeIntervals.hpp"
#include "debug.h"
#include "dynamicobstacles.h"
#include "structs.h"
#include <cmath>
#include <cstddef>


bool SafeIntervals::isSafe(int i, int j, double t) const{
    auto loc = get_safe_intervals(i, j, t).first; // first safe interval starting after t
    return (loc->first <= t && loc->second >= t); // otherwise safe if in the one earlier.
}

boost::container::flat_set<std::pair<double, double>> SafeIntervals::get_safe_intervals(int i, int j) const{
    size_t ind = get_index(i, j);
    return _safe_intervals[ind];
}

std::pair<boost::container::flat_set<std::pair<double, double>>::iterator, boost::container::flat_set<std::pair<double, double>>::iterator> SafeIntervals::get_safe_intervals(int i, int j, double beginning) const{
    auto key = std::pair<double, double>(beginning, beginning);
    auto si = get_safe_intervals(i, j);
    auto loc = si.lower_bound(key); 
    if (loc == si.begin()){ //nothing earlier means unsafe
        return std::pair<boost::container::flat_set<std::pair<double, double>>::iterator, boost::container::flat_set<std::pair<double, double>>::iterator>(loc, si.end());
    }
    --loc;
    return std::pair<boost::container::flat_set<std::pair<double, double>>::iterator, boost::container::flat_set<std::pair<double, double>>::iterator>(loc, si.end());
}

SafeInterval SafeIntervals::getSafeInterval(int i, int j, double beginning) const{
    auto safe_intervals = get_safe_intervals(i, j, beginning);
    if (safe_intervals.first == safe_intervals.second || beginning < safe_intervals.first->first){
        return SafeInterval(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
    }
    return SafeInterval(safe_intervals.first->first, safe_intervals.first->second);
}

std::pair<boost::container::flat_set<std::pair<double, double>>::iterator, boost::container::flat_set<std::pair<double, double>>::iterator> SafeIntervals::get_safe_intervals(int i, int j, double beginning, double ending) const{
    auto key = std::pair<double, double>(ending, ending);
    auto si = get_safe_intervals(i, j);
    auto loc = si.upper_bound(key);
    auto retval = get_safe_intervals(i, j, beginning); 
    retval.second = loc;
    return retval;
}


SafeIntervals::SafeIntervals(Map map, const DynamicObstacles& obstacles, double agent_size) : map(map){
    // precompute safe intervals
    auto col_interval = std::pair<double, double>(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    std::vector<double> new_intervals_begin = std::vector<double>();
    std::vector<double> new_intervals_end = std::vector<double>();

    _safe_intervals.clear();
    for (int i=0; i<map.height; i++){
        for (int j=0; j < map.width; j++){
            _safe_intervals.emplace_back(boost::container::flat_set<std::pair<double, double>>());
        }
    }
    for (int i=0; i<map.height; i++){
        for (int j=0; j < map.width; j++){
            if (map.CellIsObstacle(i, j)){
                continue;  // no safe intervals if static obstacle
            }
            auto * inter_holder = new std::vector<std::pair<double, double>>();
            int interval_key = get_index(i, j);
            DEBUG_MSG(interval_key);
            int num_obs = obstacles.getNumberOfObstacles();
            for (int ind=0; ind<num_obs; ind++ ){
                const auto& sections = obstacles.getSections(ind);
                double obstacle_size = obstacles.getSize(ind);
                for (int section_ind = 1; section_ind<sections.size(); section_ind++){
                    collision_interval(i, j, sections[section_ind-1], sections[section_ind], agent_size, obstacle_size, col_interval);
                    if (!std::isnan(col_interval.first)){
                        inter_holder->emplace_back(col_interval);
                    }
                }
            }
            _safe_intervals[interval_key] = boost::container::flat_set<std::pair<double, double>>(inter_holder->begin(), inter_holder->end());

            // put final resting unsafe interval

            combine_intervals(_safe_intervals[interval_key], new_intervals_begin, new_intervals_end); 
            invert_intervals(_safe_intervals[interval_key], new_intervals_begin, new_intervals_end);
            DEBUG_MSG_NO_LINE_BREAK(i);
            DEBUG_MSG_NO_LINE_BREAK(" ");
            DEBUG_MSG_NO_LINE_BREAK(j);
            DEBUG_MSG_NO_LINE_BREAK(" ");
            DEBUG_MSG(_safe_intervals[interval_key].size());
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
