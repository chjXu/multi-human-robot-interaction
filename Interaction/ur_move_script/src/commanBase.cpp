#include <ur_move_script/commanBase.h>
#include <iostream>
#include <cmath>
using namespace std;

namespace commonFunction{

double compute_Euclidean_distance(Human_Joint &point_1, double &x, double &y, double &z)
{
    return std::sqrt((point_1.x - x) * (point_1.x - x) + 
                     (point_1.y - y) * (point_1.y - y) + 
                     (point_1.z - z) * (point_1.z - z));
}

}