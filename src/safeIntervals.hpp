#pragma once

#include "dynamicobstacles.h"
#include "structs.h"
#include <cstddef>
#include "map.h"
#include <boost/container/flat_set.hpp>

class Map;

class SafeIntervals{
    private:
        std::vector<boost::container::flat_set<std::pair<double, double>>> _safe_intervals;
        Map map;
        inline int get_index(int i, int j) const{return i * (int)map.width + j;}
    public:
        SafeIntervals(){};
        SafeIntervals(Map map, const DynamicObstacles& obstacles, double agent_size); // populates safe intervals
        bool isSafe(int i, int j, double t) const;
        SafeInterval getSafeInterval(int i, int j, double beginning) const;
        boost::container::flat_set<std::pair<double, double>> get_safe_intervals(int i, int j) const; // return safe intervals at location.
        std::pair<boost::container::flat_set<std::pair<double, double>>::iterator, boost::container::flat_set<std::pair<double, double>>::iterator> get_safe_intervals(int i, int j, double beginning) const; // now get rid of any that end before beginning.
        std::pair<boost::container::flat_set<std::pair<double, double>>::iterator, boost::container::flat_set<std::pair<double, double>>::iterator> get_safe_intervals(int i, int j, double beginning, double ending) const; // and start after ending. 
        void dump(const std::string & fn) const;
};


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