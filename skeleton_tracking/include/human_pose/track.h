#pragma once
#include <iostream>
#include <queue>
#include <human_pose/pose.h>
#include <pthread.h>
#include <mutex>
#include <list>
#include <assert.h>
#include <iterator>
#include <typeinfo>
#include <human_pose/skeletonPair.h>

using namespace std;
const int frameNum = 60;

namespace HumanPose
{

// 每一帧中每个姿态的信息
struct Pose3DWithId
{
    bool paired = false;
    vector<HumanPose::Pose_3d> Pose3D;
    int ID = -1;
};

// 每一帧的所有3D姿态信息
struct Skeletons
{
    int frame_index = 0;
    vector<Pose3DWithId> skeletons_3d;
};


class Track
{
public:
    list<SkeletonPair> skeleton_pair;
    Track():frame_index(1), paired_frame(0), search_frame(0)
    {}
    ~Track(){}

    // 添加姿态
    vector<vector<int>> tracking(vector<HumanPose::Pose_Info>& framepose, int operator_index);

    // 生成配对
    void generateSkeletonsPair(Skeletons &cur_frame, Skeletons& search_frame);

    // 融合
   vector<vector<int>> skeletonsAssociate();

    // 计算置信度    
    bool getSkeletonsCorrespondence(SkeletonPair& pair, double threshold=0.5);

    // 计算欧式距离
    double getDistance(Pose_3d& joint_1, Pose_3d& joint_2);

    // 计算IOU
    vector<int> getIOU(vector<HumanPose::Pose_3d>& pose_3d);

    // update ID
    void setTrackingID(Skeletons &cur_frame, Skeletons& search_frame, vector<vector<int>>& skele_as);
    void setTrackingID(vector<vector<int>>& skele_as);

    // updata Skeletons
    void updateSkeletons();


private:
    int frame_index;
    pthread_mutex_t track_mutex;
    vector<Pose3DWithId> trackPose; //要被跟踪的帧中的所有姿态
    vector<Skeletons> queue_pose; //所有帧之间的所有姿态
    vector<Pose3DWithId> results;

    int paired_frame;   //待匹配帧
    int search_frame;   //搜索帧
};

struct track_args
{
    Track *obj;
    list<SkeletonPair>::iterator pair;
};

}