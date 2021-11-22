#pragma once
#include "../gl_const.h"
#include "structs.h"
#include <vector>
#include "../tinyxml2.h"
#include <string>
#include <iostream>
#include <cmath>

class RTDynamicObstacles
{
private:
    std::vector<RTobstacle> obstacles;
public:
    RTDynamicObstacles();
    bool getObstacles(const char* fileName);
    std::vector<RTNode> getSections(int num) const;
    double getSize(int num) const;
    double getMSpeed(int num) const;
    std::string getID(int num) const;
    int getNumberOfObstacles() const { return obstacles.size(); }
};
