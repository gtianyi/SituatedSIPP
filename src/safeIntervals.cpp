#include "safeIntervals.hpp"
#include "dynamicobstacles.h"
#include <cstddef>

bool SafeIntervals::isSafe(size_t i, size_t j, double t) const{
    auto loc = get_safe_intervals(i, j, t).first; // first safe interval starting after t
    return (loc->first <= t && loc->second >= t); // otherwise safe if in the one earlier.
}

std::set<std::pair<double, double>> SafeIntervals::get_safe_intervals(size_t i, size_t j) const{
    size_t ind = get_index(i, j);
    return _safe_intervals[ind];
}

std::pair<std::set<std::pair<double, double>>::iterator, std::set<std::pair<double, double>>::iterator> SafeIntervals::get_safe_intervals(size_t i, size_t j, double beginning) const{
    auto key = std::pair<double, double>(beginning, beginning);
    auto si = get_safe_intervals(i, j);
    auto loc = si.lower_bound(key); 
    if (loc == si.begin()){ //nothing earlier means unsafe
        return std::pair<std::set<std::pair<double, double>>::iterator, std::set<std::pair<double, double>>::iterator>(loc, si.end());
    }
    return std::pair<std::set<std::pair<double, double>>::iterator, std::set<std::pair<double, double>>::iterator>(loc - 1, si.end());
}

std::pair<std::set<std::pair<double, double>>::iterator, std::set<std::pair<double, double>>::iterator> SafeIntervals::get_safe_intervals(size_t i, size_t j, double beginning, double ending) const{
    auto key = std::pair<double, double>(ending, ending);
    auto si = get_safe_intervals(i, j);
    auto loc = si.upper_bound(key);
    auto retval = get_safe_intervals(i, j, beginning); 
    retval.second = loc;
    return retval;
}

SafeIntervals::SafeIntervals(const Map & map, const DynamicObstacles& obstacles) : map(map){
    // precompute safe intervals
    std::vector<std::set<std::pair<double, double>>> unsafe_intervals;
    for (int i=0; i<map.height; i++){
        for (int j=0; j < map.width; j++){
            if (map.CellIsObstacle(i, j)){
                continue;  // no safe intervals if static obstacle
            }
            int interval_key = get_index(i, j);
            int num_obs = obstacles.getNumberOfObstacles();
            for (int ind=0; ind<num_obs; ind++ ){
                const auto& sections = obstacles.getSections(ind);
                for (int section_ind = 1; section_ind<sections.size(); section_ind++){
                    const Node& source = sections[section_ind-1];
                    const Node& destination = sections[section_ind];  
                    unsafe_intervals[interval_key].emplace(collision_interval(i, j, source, destination));
                }
                // put final resting unsafe interval
            }
        }
    }
}

std::vector<std::pair<double, double>> collision_interval(int i, int j, const Node& source, const Node& destination, double agent_size, double obstacle_size){
    auto retval = std::vector<std::pair<double, double>>();
    double di = destination.i - source.j;
    double dj = destination.j - source.j;
    double ic = (destination.g - source.g)*(source.i - i) + source.g *(source.i - destination.i);
    double jc = (destination.g - source.g)*(source.j - j) + source.g *(source.j - destination.j);
    double a = pow(di, 2) + pow(dj, 2);
    double b = 2*(di*ic + dj*jc);
    double c = -pow(agent_size + obstacle_size, 2);
    double discriminant = pow(b, 2) - 4*a*c;
    if (discriminant >= 0){
        double shared_term = sqrt(discriminant);
        double beginning = (-b - shared_term)/(0.5*a);
        double ending = (-b + shared_term)/(0.5*a);
        if ((ending >= source.g) && beginning <= destination.g){
            retval.emplace_back(std::max(beginning, source.g), std::min(ending, destination.g));
        }
    }   
    return retval;
}