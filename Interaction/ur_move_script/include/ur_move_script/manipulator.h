#pragma once
#include <boost/scoped_ptr.hpp>
//------------kdl运动学库----------------//
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
//------------逆运动学库-----------------//
#include <trac_ik/trac_ik.hpp>  
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
//------------ur5----------------------//
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <pthread.h>
#include <cmath>
#include <iostream>
#include <vector>  //存储矫正点和目标点：矫正点（new）,目标点（实际的marker）
#include <ctime>
// #include <object_msgs/childFrameId.h>
// #include <object_msgs/childFrameList.h>

#define JOINT_NUM 6
#define PUB_RATE 20
#define TIME_STEP 1.0/PUB_RATE

using std::string;
using namespace std;
bool JointState_available;
KDL::JntArray q(6);   //当前
KDL::JntArray q_desired(6);  //期望
KDL::Frame correct_pose , desired_pose;
// vector<KDL::Frame> pose{correct_pose , desired_pose};

std::vector<std::string> joint_names(JOINT_NUM);  //机器人6个关节的名称列表
std::vector<double> joint_angles(JOINT_NUM);      //机器人6个关节的关节角度
std::vector<double> joint_speed(JOINT_NUM);       //机器人6个关节的关节速度

string mode1 = "track_obj";
string mode2 = "track_point";

class Manipulator
{
public:
    Manipulator()
    {
        string topic1 = "/ur_driver/URScript";
        string topic2 = "/joint_states";
        this->init();
        // id_sub = nh.subscribe("child_frame_id_list",3,&Manipulator::idCallback,this);
        ur_cmd_publisher = nh.advertise<std_msgs::String>(topic1, 1);
        ur_joint_publisher = nh.advertise<sensor_msgs::JointState>(topic2, 1);
        joint_state_sub = nh.subscribe(topic2, 3, &Manipulator::jointStateCallback,this);
    }

    void init();
    void jointStateCallback(const sensor_msgs::JointState& joint_state);
    void* goal_rectify__thread(void *arg);
    void listener_object();
    void listener_point();
    void sendCmd(TRAC_IK::TRAC_IK &ik_solver , int &idx);
    void track();
    bool judge(int &idx); //先测试一下
    string getModeState();
    bool isToPlace();
    bool judge_update();
    // void idCallback(const object_msgs::childFrameList& id_list);

private:
    pthread_t rectify_thread;

    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_br;
    
    ros::Publisher ur_cmd_publisher;  // /ur_driver/URScript：话题名称，类型是std_msgs::String，允许我们向该话题发布URScript脚本命令
    ros::Publisher ur_joint_publisher; // /joint_states：话题名称，类型是：sensor_msgs::JointState，实现机械臂的运动，就是向话题joint_states添加有关关节的消息
    ros::Subscriber joint_state_sub;
    ros::Subscriber id_sub;

    std_msgs::String cmd_msg;
    double Kp = 2.0;
    double Td = 0.2;
    double max_speed = 1.5;   //最大速度

    string urdf_param = "/robot_description";
    string base = "base";
    string tip = "tool0";

    tf::StampedTransform transform_correct , transform_target , transform_current;
    tf::StampedTransform control_speed;
    tf::StampedTransform place_point , init_point ;
    tf::StampedTransform marker_origin , marker_current;  //用来判断是否更新矫正点和抓取点
    bool tf_available = false;

    vector<tf::StampedTransform> correct_target , place_init , correct_target_origin;
    // object_msgs::childFrameList child_id_;
 
    string mode;           //包括track_obj track_point
    bool toPlace , update;

    int count = 0;

    // object_msgs::childFrameList idList;
    // vector<object_msgs::childFrameId> id_vec;

};