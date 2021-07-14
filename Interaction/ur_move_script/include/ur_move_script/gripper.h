#pragma once
#include <iostream>
#include <string>
#include <ros/ros.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>
#include <byp80/ByStatus.h>
#include <byp80/ShutdownGripper.h>
#include <byp80/RestartGripper.h>
#include <byp80/MoveTo.h>
#include <byp80/GetStatus.h>
#include <byp80/GetCalibrated.h>
#include <byp80/CalibrateGripper.h>



class Gripper
{
public:
    Gripper(){
        // this->init();
        // this->status_client = nh.serviceClient<byp80::GetStatus>("get_status");
        // this->calibrated_client = nh.serviceClient<byp80::CalibrateGripper>("calibrate_gripper");
        this->move_to_client = nh.serviceClient<byp80::MoveTo>("move_to");

        // status_client.call(srv_getstatus);
        // calibrated_client.call(srv_calibarate);
        move_to_client.call(srv_move);
    }
    // void init();   //设置放置的目标点 、 放置之后的回到的点
    void clamp();  //夹紧爪子
    void loose();  //松开

private:
    ros::NodeHandle nh;
    ros::ServiceClient status_client;
    ros::ServiceClient calibrated_client;
    ros::ServiceClient move_to_client;
	byp80::GetStatus srv_getstatus;
	byp80::CalibrateGripper srv_calibarate;
	byp80::MoveTo srv_move;

    // tf::Transform place_point;
    // tf::Transform init_point; 

    // tf::TransformBroadcaster br;
    // tf::TransformListener listen;
};
