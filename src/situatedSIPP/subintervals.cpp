#include "subintervals.hpp"
#include <cmath>
#include <algorithm> 
double SubInterval::ht(double t) const{
    if (t > ending){
        return INFINITY;
    }
    if (t >= beginning){
        return h; 
    }
    return h + beginning - t;
}

void SetOfSubIntervals::add(double start, double end, double h, double shift){
    double s = std::max(beginning, (start - shift));
    double e = std::min(ending, (end - shift));
    if(s <= e){
        subintervals.emplace_back(s, e, h);
    }
}

void SetOfSubIntervals::prune_expired(double t){
    std::vector<SubInterval> new_subints;
    for (auto si: subintervals){
        if (si.ending >= t){
            new_subints.emplace_back(si);
        }
    }
    subintervals = new_subints;
}

double SetOfSubIntervals::ht(double t) const{
    double retval = INFINITY;
    if ((t >= beginning) && (t <= ending)){
        for (auto si: subintervals){
            double sih = si.ht(t);
            if (sih < retval){
                retval = sih;
            }
        }
    }
    return retval;
}