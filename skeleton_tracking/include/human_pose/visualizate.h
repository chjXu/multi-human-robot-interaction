#pragma once

#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <human_pose/pose.h>
#include <human_pose/track.h>
#include <vector>
#include <queue>
#include <map>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

using namespace std;

namespace HumanPose{

const vector<vector<int>> init_pose = {{746, 291}, {737, 206}, {746, 535}, {830, 291}, {939, 299}, {1067, 274}, {796, 535}, {822, 662}, {864, 788}, 
                       {653, 281}, {527, 265}, {409, 232}, {696, 535}, {712, 679}, {729, 796}, {721, 189}, {755, 188}, {696, 198}, {788, 198}};
const vector<int> body_edge = {0, 1, 0, 2, 0, 3, 3, 4, 4, 5, 0, 9, 9, 10, 10, 11, 2, 6, 6, 7, 7, 8, 2, 12, 12, 13, 13, 14};


struct PoseTF{
    PoseTF(){
        joint.resize(7);
        //smooth_queue.resize(3);
    }
    
    vector<tf::Transform> joint;
    int person_id;
};

class Visualizate
{
public:
    cv::Mat image;
    Eigen::Matrix3d rot;
    Eigen::Matrix<double, 3, 1> trans; 
    cv::Mat cameraMatrix;

    Visualizate(ros::Publisher& pub):x_previous(0.0){
        pose_pub = &pub;

        body_color.resize(5);
        body_color[0] = cv::Scalar(0, 0, 255);
        body_color[1] = cv::Scalar(255, 255, 0);
        body_color[2] = cv::Scalar(255, 0, 255);
        body_color[3] = cv::Scalar(0, 255, 255);
        body_color[4] = cv::Scalar(0, 255, 0);

        //骨骼
        body_edge = {0, 1, 0, 2, 0, 3, 3, 4, 4, 5, 0, 9, 9, 10, 10, 11, 2, 6, 6, 7, 7, 8, 2, 12, 12, 13, 13, 14};

        // 关节名称
        joint_name.resize(7);
        joint_name[0] = "/neck";
        joint_name[1] = "/lWrist";
        joint_name[2] = "/lElbow";
        joint_name[3] = "/lShoulder";
        joint_name[4] = "/rWrist";
        joint_name[5] = "/rElbow";
        joint_name[6] = "/rShoulder";

        br = new tf::TransformBroadcaster;
    }
    ~Visualizate(){
        delete br;
    }
    // add相机信息
    void addCameraInfo(Eigen::Matrix3d& rot, Eigen::Matrix<double, 3, 1>& trans, cv::Mat& cameraMatrix);

    // 投影
    cv::Point2d project_from_world(const Pose_3d& pose_3d);
 
    // 显示原始2D姿态
    void show2DPose(vector<HumanPose::Pose_Info>& ,int frame_index, int fps);
    map<int, vector<Pose_3d>> show2DPose(vector<HumanPose::Pose_Info>& person_info, vector<vector<int>> &new_index_id,int frame_index);

    // 添加配对结果
    void addPosesAssociate(const vector<vector<int>>& poses_ass);

    // 添加并发布3D姿态计算结果
    void Publish3DPoses(vector<vector<HumanPose::Pose_3d>>& pose_3d);
    void Publish3DPoses(vector<HumanPose::Pose_Info>& pose_3d);
    void Publish3DPosesTF(vector<HumanPose::Pose_Info>& pose_3d);
    void Publish3DPosesTF(std::map<int, vector<Pose_3d>> results);


    void averagePose(Pose_3d& current_pose, Pose_3d& previous_pose, double alpha);
    void smooth(std::map<int, vector<Pose_3d>>& resluts);
    vector<HumanPose::Pose_3d> smooth(vector<HumanPose::Pose_3d>& pose_3d);

    // 设置TF
    PoseTF setPosesTF(vector<HumanPose::Pose_3d>& pose, int id);

    // 划线
    void addLine(visualization_msgs::Marker& line_list, vector<HumanPose::Pose_3d>& pose, int a, int b);

    void recognizating(vector<HumanPose::Pose_Info>& person_info);
private:
    vector<cv::Scalar> body_color;
    //vector<vector<int>> temp_pose_ass;

    ros::Publisher *pose_pub;
    vector<double> body_edge;
    vector<std::string> joint_name;
    tf::TransformBroadcaster *br;
    vector<PoseTF> TFPose;

    queue<std::map<int, vector<Pose_3d>>> queue_results;
    float x_previous;

    std::map<int, vector<Pose_3d>> pre_results;
    vector<std::map<int, vector<Pose_3d>>> smooth_queue;
};



}