#pragma once

#include "dynamicobstacles.h"
#include "structs.h"
#include <cstddef>
#include "map.h"
#include <boost/container/flat_set.hpp>

class Map;


inline void collision_interval(int i, int j, const Node& source, const Node& destination, double agent_size, double obstacle_size, std::pair<double, double> & retval){
    retval.first = std::numeric_limits<double>::quiet_NaN();
    double duration = (destination.g - source.g);
    double one_over_duration = 1/duration;
    double di = (destination.i - source.i)*one_over_duration;
    double dj = (destination.j - source.j)*one_over_duration;
    double ic = -di*source.g + source.i - i;
    double jc = -dj*source.g + source.j - j;
    double a = di*di + dj*dj;
    double b = 2*(di*ic + dj*jc);
    double size = agent_size + obstacle_size;
    double c = ic*ic + jc*jc - size*size;
    double discriminant = b*b - 4*a*c;
    if (discriminant >= 0){
        double shared_term = sqrt(discriminant);
        double one_over_2a = 1/(2*a);
        double beginning = (-b - shared_term)*one_over_2a;
        double ending = (-b + shared_term)*one_over_2a;
        if ((ending >= source.g) && beginning <= destination.g){
            retval.first = std::max(beginning, source.g);
            retval.second = std::min(ending, destination.g);
        }
    }   
}


inline void invert_intervals(boost::container::flat_set<std::pair<double, double>>& intervals, std::vector<double>& new_intervals_begin, std::vector<double>& new_intervals_end){
    if (intervals.empty()){
        intervals.emplace_hint(intervals.end(), 0.0, std::numeric_limits<double>::infinity());
        return;
    }

    new_intervals_begin.clear();
    if (new_intervals_begin.capacity() < intervals.size()){
        new_intervals_begin.reserve(intervals.size());
    }
    new_intervals_end.clear();
    if (new_intervals_end.capacity() < intervals.size()){
        new_intervals_end.reserve(intervals.size());
    }

    double safe_interval_start = 0.0;
    for (auto interval: intervals){
        if (interval.first > 0.0){
            new_intervals_begin.emplace_back(safe_interval_start);
            new_intervals_end.emplace_back(interval.first);
            safe_interval_start = interval.second;
        }
        
    }

    if (std::isfinite(safe_interval_start)){ // add safe interval to infinity if appropriate
        new_intervals_begin.emplace_back(safe_interval_start);
        new_intervals_end.emplace_back(std::numeric_limits<double>::infinity());
    }

    intervals.clear();
    for (int i = 0; i < new_intervals_begin.size(); i++){
        intervals.emplace_hint(intervals.end(), new_intervals_begin[i], new_intervals_end[i]);
    }
}

inline void combine_intervals(boost::container::flat_set<std::pair<double, double>>& intervals, std::vector<double>& new_intervals_begin, std::vector<double>& new_intervals_end){
    if (intervals.empty()){
        return;
    }
    new_intervals_begin.clear();
    if (new_intervals_begin.capacity() < intervals.size()){
        new_intervals_begin.reserve(intervals.size());
    }
    new_intervals_end.clear();
    if (new_intervals_end.capacity() < intervals.size()){
        new_intervals_end.reserve(intervals.size());
    }
    new_intervals_begin.emplace_back(intervals.begin()->first);
    new_intervals_end.emplace_back(intervals.begin()->second);
    auto interval = intervals.begin();
    ++interval;
    while(interval != intervals.end()){
        if (interval->first <= new_intervals_end[new_intervals_end.size()-1]){
            new_intervals_end[new_intervals_end.size()-1] = std::max(new_intervals_end[new_intervals_end.size()-1], interval->second);
        }
        else{
            new_intervals_begin.emplace_back(interval->first);
            new_intervals_end.emplace_back(interval->second);
        }
        ++interval;
    }
    intervals.clear();
    for (int i = 0; i < new_intervals_begin.size(); i++){
        intervals.emplace_hint(intervals.end(), new_intervals_begin[i], new_intervals_end[i]);
    }

}

inline void lazy_compute_safe_intervals(int i, int j, const DynamicObstacles& obstacles, const Map& map, double agent_size, boost::container::flat_set<std::pair<double, double>> * retval){
    if (map.CellIsObstacle(i, j)){
        return;
    }
    auto col_interval = std::pair<double, double>(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    auto inter_holder = std::vector<std::pair<double, double>>();
    std::vector<double> new_intervals_begin = std::vector<double>();
    std::vector<double> new_intervals_end = std::vector<double>();

    int num_obs = obstacles.getNumberOfObstacles();
    for (int ind=0; ind<num_obs; ind++ ){
        const auto& sections = obstacles.getSections(ind);
        double obstacle_size = obstacles.getSize(ind);
        for (int section_ind = 1; section_ind<sections.size(); section_ind++){
            collision_interval(i, j, sections[section_ind-1], sections[section_ind], agent_size, obstacle_size, col_interval);
            if (!std::isnan(col_interval.first)){
                inter_holder.emplace_back(col_interval);
            }
            if ((section_ind == sections.size()-1) && (sections[section_ind].i == i) && (sections[section_ind].j == j)){
                inter_holder.emplace_back(sections[section_ind].g, std::numeric_limits<double>::infinity());
            }
        }
    }
    for (auto inter :inter_holder){
        retval->insert(inter);
    }
    combine_intervals(*retval, new_intervals_begin, new_intervals_end); 
    invert_intervals(*retval, new_intervals_begin, new_intervals_end);
    DEBUG_MSG_NO_LINE_BREAK(i);
    DEBUG_MSG_NO_LINE_BREAK(" ");
    DEBUG_MSG_NO_LINE_BREAK(j);
    DEBUG_MSG_NO_LINE_BREAK(" ");
    DEBUG_MSG(retval->size());
}


class SafeIntervals{
    private:
        std::vector<boost::container::flat_set<std::pair<double, double>>* > _safe_intervals;
        std::vector<bool> _generated_safe_intervals;
        const Map& map;
        const DynamicObstacles& obstacles;
        double agent_size;
        inline int get_index(int i, int j) const{return i * (int)map.width + j;}
    public:
        SafeIntervals(const Map& map, const DynamicObstacles& obstacles, double agent_size); // populates safe intervals
        ~SafeIntervals(){
            for (int i = 0; i<_safe_intervals.size(); i++){
                _safe_intervals[i]->clear();
                delete _safe_intervals[i];
            }
        }
        bool isSafe(int i, int j, double t);
        SafeInterval getSafeInterval(int i, int j, double beginning);
        boost::container::flat_set<std::pair<double, double>> get_safe_intervals(int i, int j); // return safe intervals at location.
        boost::container::flat_set<std::pair<double, double>> get_safe_intervals(int i, int j, double beginning); // now get rid of any that end before beginning.
        boost::container::flat_set<std::pair<double, double>> get_safe_intervals(int i, int j, double beginning, double ending); // and start after ending. 
        void dump(const std::string & fn) const;
};



