#include "subintervals.h"
#include <algorithm>
#include <numeric>
SubIntervals::SubIntervals(double _start, double _end, const std::vector<SubIntervals>& children): start(_start), end(_end){
    auto total_length = std::accumulate(children.begin(), children.end(), 0, [](auto acc, const SubIntervals& c){return acc + c.size();});
    auto child_intervals = std::vector<SubInterval>(total_length);
    for(const auto& child: children){
        for(auto it = child.relevant_start(_end); it != child.subintervals_end();it++){//this is wrong, fix.

        }
    }
    std::sort(child_intervals.begin(), child_intervals.end(), std::greater<>());
    const SubInterval& incumbant = SubInterval()
}