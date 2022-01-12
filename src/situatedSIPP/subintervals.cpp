#include "subintervals.h"
#include <algorithm>
#include <numeric>

void SubIntervals::add(const SubInterval &si, double cost){
    double new_start = si.start  - cost;
    double new_end = si.end - cost;
    double new_h = si.h + cost;
    if ((new_h <= h(new_start))||(new_h <= h(new_end))){ // check for strong dominance
        SubInterval new_si = SubInterval(new_start, new_end, new_h);
        auto location = subintervals.lower_bound(new_si);
        while (location != subintervals.end()){
            if (new_si.strongly_dominates(*location)){
                location = subintervals.erase(location);
            }
            else{
                location++;
            }
        }
        subintervals.insert(new_si);
    }
}


SubIntervals::SubIntervals(double _start, double _end, const std::vector<SubIntervals>& children, const std::vector<double>& children_costs): start(_start), end(_end){
    
}


