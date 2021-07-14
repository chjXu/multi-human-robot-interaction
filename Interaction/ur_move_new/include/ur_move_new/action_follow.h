#pragma once

#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;

class Action{
public:
    Action():orient(-0.488,0.86,-0.05,-0.056){
    }
    ~Action(){
    }

    void init();
    tf::Quaternion getIMUorient();
    void follow_hand();
private:
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster br;

    tf::StampedTransform base_tool;
    tf::StampedTransform rWrist_order;
    vector<tf::StampedTransform> desired_goal_list;
    tf::StampedTransform rWrist_imu;
    tf::Vector3 offset;
    tf::Transform desired_position;
    tf::Quaternion orient;
};