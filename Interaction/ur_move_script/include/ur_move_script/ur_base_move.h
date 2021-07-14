#include <ros/ros.h>
#include <iostream>
#include <open_ur5_msgs/ref.h>
#include <open_ur5_msgs/cur.h>
#include <open_ur5_msgs/control.h>

using namespace std;

class UR
{
public:
    UR(){
        state_ref.q_ref[0] = 10.0;
        state_ref.v_ref[0] = 1.0;
        state_cur.q_cur[0] = 0.0;
        state_cur.v_cur[0] = 0.0;

        joint_ref_pub = nh.advertise<open_ur5_msgs::ref>("/ur5_ref_joint_state", 1);
        joint_cur_pub = nh.advertise<open_ur5_msgs::cur>("/ur5_cur_joint_state", 1);
        control_var_sub = nh.subscribe("/ur5_joint_accle", 1, &UR::controlCallback, this);
    }

    void controlCallback(const open_ur5_msgs::control::ConstPtr &msg)
    {
        acc = msg->u[0];
    }

    void updata()
    {
        if(fabs(state_ref.q_ref[0] - state_cur.q_cur[0]) < 0.1){
            state_cur.q_cur[0] = state_ref.q_ref[0];
            state_cur.v_cur[0] = state_ref.v_ref[0];
        }
        else{
            state_cur.q_cur[0] += state_cur.v_cur[0] * sample_time + 0.5 * acc * sample_time * sample_time;
            state_cur.v_cur[0] += acc * sample_time; 
        }
    }

    void pub()
    {
        joint_ref_pub.publish(state_ref);
        joint_cur_pub.publish(state_cur);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher joint_ref_pub, joint_cur_pub;
    ros::Subscriber control_var_sub;

    double acc;
    double sample_time = 0.1;
    open_ur5_msgs::cur state_cur;
    open_ur5_msgs::ref state_ref;
};