#pragma once
#include <limits>
#include <map>
#include <vector>

struct RTNode;

struct SubInterval{
    double beginning;
    double ending;
    double h;
    SubInterval(double s, double e, double ctg) : beginning(s), ending(e), h(ctg){};
    bool operator==(const SubInterval& si) const{
        return (beginning == si.beginning) && (ending == si.ending) && (h == si.h);
    }
    double ht(double t) const;
    void debug() const;
};

class SetOfSubIntervals{
    public:
        double beginning;
        double ending;
        std::vector<const RTNode *> nodes;
        std::vector<SubInterval> subintervals;
        SetOfSubIntervals(double _start = 0.0, double _end = std::numeric_limits<double>::infinity()) : beginning(_start), ending(_end){};
        void add(double start, double end, double h, double shift, const RTNode* node);
        double ht(double t) const;
        std::pair<double, const RTNode*> htn(double t) const;
        void prune_expired(double t);
        void debug() const;
};
