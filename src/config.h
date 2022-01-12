#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include "tinyxml2.h"
#include <string>
#include "gl_const.h"
#include <algorithm>
#include <sstream>


class Config
{
public:
    Config();
    int loglevel;
    int connectedness;
    bool allowanyangle;
    bool planforturns;
    double timelimit;
    int rescheduling;
    double inflatecollisionintervals;
    int initialprioritization;
    double startsafeinterval;
    double additionalwait;
    double focal_weight;
    double h_weight;
    bool use_focal;
    bool use_likhachev;
    int algtype;
    double weight;
    std::string logfilename;
    std::string logpath;
    int fixedlookahead;
    std::string learningalgorithm;
    std::string expansionalgorithm;
    std::string decisionalgorithm;
    int dynmode;
    unsigned long maxNumOfIntervalsPerMove;
    double unitWaitDuration;
    bool isUnitWaitRepresentation;
    bool getConfig(const char* fileName);
};

#endif
