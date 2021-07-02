#pragma once

namespace HumanPose
{

class SkeletonPair{
public:
    int cur_frame_index; //当前帧索引
    int sea_frame_index; //搜索帧索引

    int index_1; //当前帧中骨架的索引
    int index_2; //其他帧中骨架的索引

    double delta;

    SkeletonPair(){
        delta = 1000;
    }

    static bool comp(SkeletonPair& skeleton_1, SkeletonPair& skeleton_2){
        return skeleton_1.delta < skeleton_2.delta;
    }
private:

};

}