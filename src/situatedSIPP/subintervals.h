#pragma once
#include <boost/container/flat_map.hpp>
#include <limits>
#include <vector>
#include <pair>

struct SubInterval{
    double start;
    double end;
    double h;
    SubInterval(double _start, double _end, double _h):start(_start),end(_end),h(_h){}
    bool operator>(const SubInterval& right) const{
       return (end > right.end);
    }

    double ht(double t){
        if(t > end){
            return std::numeric_limits<double>::infinity();
        }
        if (t < start){
            return h + start - t;
        }
        return h;
    }
};

class SubIntervals{
    private:
        double start;
        double end;
        boost::container::flat_map<double, SubInterval, std::greater<>> subintervals;
    public:
        SubIntervals(double _start, double _end, const std::vector<SubIntervals>& children);
        SubIntervals(double _start, double _end, double h) : start(_start), end(_end){
            subintervals.emplace(_end, SubInterval(start, _end, h));
        }
        double h(double t) const;
        auto inline size() const{
            return subintervals.size();
        }
        auto inline relevant_start(double t) const{
            return subintervals.lower_bound(t);
        }
        auto inline subintervals_end() const{
            return subintervals.end();
        }
};