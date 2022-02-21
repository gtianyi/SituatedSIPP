#pragma once
#include <boost/container/container_fwd.hpp>
#include <boost/multi_index/tag.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <map>
#include <vector>



struct SubInterval{
    double beginning;
    double ending;
    double h;
    SubInterval(double s, double e, double ctg) : beginning(s), ending(e), h(ctg){};
    
    bool operator==(const SubInterval& si) const{
        return (beginning == si.beginning) && (ending == si.ending) && (h == si.h);
    }
    double ht(double t) const;
};

class SetOfSubIntervals{
    public:
        double beginning;
        double ending;
        std::vector<SubInterval> subintervals;
        SetOfSubIntervals(double _start, double _end) : beginning(_start), ending(_end){};
        void add(double start, double end, double h, double shift);
        double ht(double t);
};
