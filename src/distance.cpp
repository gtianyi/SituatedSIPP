#include <cmath>
#include <iostream>
auto pairwiseMinimumDistance(double x0_initial, double y0_initial, double x0_goal, double y0_goal, double t0_initial, double t0_goal, double x1_initial, double y1_initial, double x1_goal, double y1_goal, double t1_initial, double t1_goal) -> double{
  double dx_at_start = x1_initial - x0_initial;
  double dy_at_start = y1_initial - y0_initial;
  double s0x = (x0_goal - x0_initial)/(t0_goal - t0_initial);
  double s1x = (x1_goal - x1_initial)/(t1_goal - t1_initial);
  double s0y = (y0_goal - y0_initial)/(t0_goal - t0_initial);
  double s1y = (y1_goal - y1_initial)/(t1_goal - t1_initial);
  double ax = dx_at_start - s1x*t1_initial + s0x*t0_initial;
  double ay = dy_at_start - s1y*t1_initial + s0y*t0_initial;
  return -(ax*(s1x-s0x) + ay*(s1y-s0y))/(pow(s1x-s0x, 2.0) + pow(s1y-s0y, 2.0));
}
/*
int main() {
  std::cout << pairwiseMinimumDistance(-10.0, 1.0, 0.0, 2.0, 0.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 10.0) << "\n";
  return 0;
}
*/
