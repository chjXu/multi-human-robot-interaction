#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>


#include <human_pose_msgs/Human.h>
#include <human_pose_msgs/HumanList.h>
#include <human_pose_msgs/PointWithProb.h>

namespace HumanPose
{

//用于保存从网络获取的3D关节信息
struct Pose_3d
{
    float x;
    float y;
    float z;
    float p;
    bool available = false;
};


class Pose_Info
{
public:
    std::vector<Pose_3d> pose_3d; //所有关节点组成的3D姿态
    int label; //姿态的id信息，方便后来匹配使用
    int action; 
    bool is_opeator;

    Pose_Info():action(-1), is_opeator(false){
        pose_3d.resize(15);

        for(int i=0; i< pose_3d.size(); ++i){
            pose_3d[i].x = 0.0;
            pose_3d[i].y = 0.0;
            pose_3d[i].z = 0.0;
            pose_3d[i].p = 0.0;
            pose_3d[i].available = false;
        }
    }

    void setLabel(const int id){
        this->label = id;
    }

    //得到从神经网络输出的3D姿态，用作匹配
    void setPose(human_pose_msgs::HumanList human_list, int id);
    void setAction(int action);
    void trangulation(Eigen::Matrix3d rot, Eigen::Matrix<double, 3, 1> trans);
    int getLabel() const{
        return this->label;
    }
    
protected:
private:   
};

}