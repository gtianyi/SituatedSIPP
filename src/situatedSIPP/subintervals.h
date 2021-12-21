#pragma once
#include <boost/multi_index_container.hpp>

struct SubInterval{
    double start;
    double end;
    double h;

    SubInterval(double _start, double _end, double _h) : start(_start), end(_end), h(_h){}

    bool operator<(const SubInterval& si) const{
            if (start < si.start){
                return true;
            }
            if (start > si.start){
                return false;
            }
            if (end > si.end){
                return true;
            }
            if (end < si.end){
                return false;
            }
            return h < si.h;
        }
    
    bool operator==(const SubInterval& si) const{
        return (start == si.start) && (end == si.end) && (h == si.h);
    }

    int strongly_dominates(const SubInterval& si) const{
        // 1 stongly dominates, 0 neither, -1 strongly dominated
        if ((h <= si.h) && (start <= si.start) && (end <= si.end)){
            return 1;
        }
        if ((si.h <= h) && (si.start <= start) && (si.end <= end)){
            return -1;
        }
        return 0;
    }
};