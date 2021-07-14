#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <thread>
#include <cmath>
#include <queue>
#include <vector>
#include <mutex>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <ur_move_new/gripper.h>

// const double SECURITY_DISTANCE = 0.4;
// const double SECURITY_SPEED = 0.4;

using namespace std;
using std::queue;
using std::vector;

class Collsion{
public:
    Collsion():SECURITY_DISTANCE(0.5), SECURITY_SPEED(0.4), weight(5.0){
        //init();
    }
    ~Collsion(){}

    //设置目标点，以及顺序
    void init();

    //判断最小距离
    int judgeJoint(vector<tf::StampedTransform>& joint_tf);

    //选择关节
    tf::StampedTransform switchJoint(int obs_index);

    void setTest(tf::StampedTransform& test);

    void collsion_avoidance(vector<tf::StampedTransform>& joint_tf, int id);
private:
    double SECURITY_DISTANCE;
    double SECURITY_SPEED;
    double weight;

    ros::NodeHandle nh;
    vector<tf::StampedTransform> obstances;

    tf::TransformBroadcaster br;
    tf::TransformListener tf_listen;
    //目标点
    vector<tf::Transform> att_goals;
    tf::Transform att_goal_one;
    tf::Transform att_goal_two;
    tf::Transform att_goal_middle;
    tf::Transform goal;

    tf::StampedTransform attraction;
    tf::StampedTransform repulsive;

    tf::StampedTransform base_obstacle;
    tf::StampedTransform base_tool;
    tf::StampedTransform tool_obstacle;
    tf::StampedTransform test;
    queue<int> target_queue;
    int goal_index;

    tf::Transform repu_goal;
    tf::Vector3 oposition;
    tf::Vector3 nopsition;
    tf::Vector3 n_o_distance;

	tf::Transform goal1, goal2, goal3, goal4, goal5, goal6;
	tf::Transform place_goal;

    Gripper gripper;
};