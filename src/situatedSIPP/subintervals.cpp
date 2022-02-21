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
    subintervals.emplace_back(std::max(beginning, (start - shift)), std::min(ending, (end - shift)), h);
}

double SetOfSubIntervals::ht(double t) const{
    double retval = INFINITY;
    for (auto si: subintervals){
        double sih = si.ht(t);
        if (sih < retval){
            retval = sih;
        }
    }
    return retval
}