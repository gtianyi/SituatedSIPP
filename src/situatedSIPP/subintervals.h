#pragma once
#include <boost/container/flat_map.hpp>
#include <set>
#include <map>
#include <limits>
#include <vector>
#include <utility>
	

struct SubInterval{
    double start;
    double end;
    double h;
    SubInterval(double _start, double _end, double _h):start(_start),end(_end),h(_h){}
    bool operator>(const SubInterval& right) const{
        return (end > right.end);
    }
    bool operator<(const SubInterval& right) const{
        return (end < right.end);
    }

    bool inline strongly_dominates(const SubInterval& other) const{
        return (other.h > ht(other.end)) && (other.h > ht(other.start));
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
        std::set<SubInterval, std::greater<>> subintervals;
        std::map<double, SubInterval> dominant;
        void update_dominant();
    public:
        SubIntervals(double _start, double _end, double _h):start(_start),end(_end){
            subintervals.emplace(_start, _end, _h);
            dominant.emplace(_end, _start, _end, _h);
        };
        SubIntervals(double _start, double _end, const std::vector<SubIntervals>& children, const std::vector<double>& children_cost);
        void add(const SubInterval & si, double cost);
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