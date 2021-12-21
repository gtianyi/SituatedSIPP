import numpy as np
import matplotlib.pyplot as plt
import sys
from openpyxl.utils.cell import get_column_letter
eps = 0.001

class SubInterval:
    def __init__(self, start, end, h):
        self.start = start
        self.end = end
        self.h = h
    
    def __lt__(self, si):
            if (self.start < si.start):
                return True
            if (self.start > si.start):
                return False
            if (self.end > si.end):
                return True
            if (self.end < si.end):
                return False
            return self.h < si.h
    def __eq__(self, si):
        return (self.start == si.start) and (self.end == si.end) and (self.h == si.h)
    
    def ht(self, t):
        if t >= self.end:
            return float("inf")
        if t < self.start:
            return self.h - (t - self.start)
        return self.h

    def strongly_dominates(self, si):
        #1 stongly dominates, 0 neither, -1 strongly dominated
        if ((self.h <= si.h) and (self.start <= si.start) and (self.end <= si.end)):
            return 1
        if ((si.h <= self.h) and (si.start <= self.start) and (si.end <= self.end)):
            return -1
        return 0
    
    def plot(self, start):
        x = [start, self.start]
        y = [self.ht(start), self.h]
        plt.plot(x, y, ":k")
        x = [self.start, self.end]
        y = [self.h, self.h]
        plt.plot(x, y, "k")
        plt.plot(x, y, "|k")

class SubIntervals:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.subintervals = [SubInterval(start, end, float("inf"))]
    def add(self, si):
        self.subintervals.append(si)
        self.subintervals.sort()

    def add(self, start, end, h):
        self.subintervals.append(SubInterval(start, end, h))
        self.subintervals.sort()
    def remove(self, si):
        self.subintervals.remove(si)
    
    def critical_point(self):
        pts = [self.start, self.end]
        for si in self.subintervals:
            pts.append(si.start)
            pts.append(si.end-eps)
            pts.append(si.end)
        return sorted(pts)

    def ht(self, t):
        mh = float("inf")
        for si in self.subintervals:
            ph = si.ht(t)
            if ph < mh:
                mh = ph
        return mh

    def h(self):
        critical_points = self.critical_point()
        critical_points = np.asarray(critical_points)
        hs = []
        for cp in critical_points:
            hs.append(self.ht(cp))
        hs = np.asarray(hs)
        redundant = [False]
        for i in range(1, len(hs)-1):
            if (hs[i-1] == hs[i])  and (hs[i] == hs[i+1]):
                redundant.append(True)
            else:
                redundant.append(False)
        redundant.append(False)
        redundant = np.asarray(redundant)
        #return critical_points[~redundant], hs[~redundant]
        return critical_points, hs

    def plot(self, num = 1000):
        acc = 1
        for si in self.subintervals[1:]:
            si.plot(self.start)
            name = get_column_letter(acc)
            plt.text(np.mean((si.start, si.end)), si.h, name,
             horizontalalignment = "center", verticalalignment = "bottom") 
            acc += 1
        cp = np.linspace(self.start, self.end, num) 
        h = np.empty(num)
        for i in range(num):
            h[i] = self.ht(cp[i])       
        plt.plot(cp, h, "r", alpha = 0.6, linewidth = 2)
    