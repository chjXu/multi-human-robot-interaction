#pragma once
#include <iostream>

namespace commonFunction
{
struct Human_Joint
{
    float x;
    float y;
    float z;
};

double compute_Euclidean_distance(Human_Joint &point_1, double &x, double &y, double &z);

}


