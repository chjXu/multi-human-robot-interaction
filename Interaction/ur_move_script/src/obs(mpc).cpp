#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <pthread.h>
#include <cmath>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include <open_ur5_msgs/cur.h>
#include <open_ur5_msgs/ref.h>
#include <open_ur5_msgs/control.h>

#include <iostream>
#include <ur_move_script/commanBase.h>

#define JOINT_NUM 6
#define PUB_RATE 100
#define TIME_STEP 1.0/PUB_RATE
const double sample_time = 0.1;


using std::string;
using namespace std;
bool JointState_available;
bool JointAccele_available;
bool HumanPose_available;

std::vector<std::string> joint_names(JOINT_NUM);
std::vector<double> joint_angles(JOINT_NUM);
std::vector<double> joint_speed(JOINT_NUM);
std::vector<double> joint_accle(JOINT_NUM);

commonFunction::Human_Joint human_rhand;

void jointStateCallback(const sensor_msgs::JointState& joint_state)
{
    joint_angles.clear();
    joint_speed.clear();
    std::vector<std::string> joint_names_recv = joint_state.name;
    for(auto it = joint_names.begin(); it !=joint_names.end(); ++it)
    {
        for(auto it_recv = joint_names_recv.begin(); it_recv != joint_names_recv.end(); ++it_recv)
        {
            if (*it_recv == *it)
            {
                int idx = it_recv - joint_names_recv.begin();
                int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);
                joint_speed.push_back(joint_state.velocity[idx]);
                break;
            }
        }
    }
    JointState_available = true;
}

// void poseCallback(const visualization_msgs::Marker &pose)
// {
//     human_rhand.x = pose.points.at(5).x;
//     human_rhand.y = pose.points.at(5).y;
//     human_rhand.z = pose.points.at(5).z;
//     cout << pose.points.size() << endl;
//     HumanPose_available = true;
// }

void controlCallback(const open_ur5_msgs::control::ConstPtr &acc)
{
    for(int i=0; i < JOINT_NUM; ++i){
        joint_accle[i] = acc->u[i];
    }
    JointAccele_available = true;
}


double one_order_filtering(double last_vel, double current_vel)
{
    return std::exp(-1./3.) * last_vel + (1.0 - std::exp(-1./3.)) * current_vel;
}

/**
 * 目标点修正线程
*/
void* goal_rectify__thread(void *arg)
{
    tf::TransformBroadcaster br;
    tf::Transform rect_goal;
    tf::TransformListener tf_listener;
    tf::StampedTransform desired_goal;

    double work_space_radius = 0.85;
    double base_radius = 0.2;

    ros::Rate rate(60);
    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base", "desired_goal", ros::Time(0), desired_goal);

            tf::Vector3 goal_position = desired_goal.getOrigin();
            tf::Vector3 rectified_position = goal_position;

            rect_goal.setRotation(desired_goal.getRotation());
            rect_goal.setOrigin(desired_goal.getOrigin());
            if (goal_position.z() < 0.15){
                rectified_position.setZ(0.15);
                rect_goal.setOrigin(rectified_position);
                ROS_WARN("goal under the working surface, setting rectified goal\n");
            }

            double desired_distance = rectified_position.length();

            if (desired_distance > work_space_radius)
            {
                double scale = work_space_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal outoff robot work space! setting rectified goal\n");
            }

            desired_distance = rectified_position.length();
            if (desired_distance < base_radius){
                double scale = base_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal too close to base! setting rectified goal\n");
            }
            br.sendTransform(tf::StampedTransform(rect_goal, ros::Time::now(), "base", "rect_goal"));
        }
        catch(tf::TransformException ex){
        }
        rate.sleep();
    }
}


int main(int argc, char** argv)
{   
	joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");
    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");

    std::string record_dir = "/home/xuchengjun/open_ros_codegen/data/state.txt";
    bool record = false;
    char* write_buffer = NULL;
    std::ofstream fs;
    if(record){
        fs.open(record_dir.c_str());
        if(fs){
            fs << "joint1, joint2, joint3, joint4, joint5, joint6, speed_1, speed_2, speed_3, speed_4, speed_5, speed_6, acc_1, acc_2, acc_3, acc_4, acc_5, acc_6\n";
        }
        write_buffer = (char*)malloc(256);
    }


    std_msgs::String cmd_msg;
    double max_speed = 0.5;

    ros::init(argc, argv, "ur_move");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

    pthread_t rectify_thread;
    pthread_create(&rectify_thread, NULL, goal_rectify__thread, NULL);

    ros::Publisher ur_cmd_publisher = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    
    ros::Publisher ur_ref_state_pub = nh.advertise<open_ur5_msgs::ref>("/ur5_ref_joint_state", 1);
    ros::Publisher ur_cur_state_pub = nh.advertise<open_ur5_msgs::cur>("/ur5_cur_joint_state", 1);
    ros::Subscriber ur_accle_sub = nh.subscribe("/ur5_joint_accle", 1, controlCallback);
    
    //订阅Marker话题
    // ros::Subscriber marker_sub = nh.subscribe("/human_pose_array", 1, poseCallback);

    string urdf_param = "/robot_description";

    string base = "base";
    string tip = "tool0";

    TRAC_IK::TRAC_IK ik_solver(base, tip, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);

    KDL::JntArray q(6);
    KDL::JntArray q_desired(6);
    KDL::Frame desired_pose;
    tf::StampedTransform transform;
    tf::StampedTransform base_tool;
    tf::StampedTransform tool_rHand;
    bool tf_available = false;

    std::vector<double> last_filter_vel(6);

    ros::Rate rate(PUB_RATE);
    std::cout << "controlling using time step at: " << TIME_STEP << std::endl;

    int count = 1;

    while(ros::ok())
    {
        try{
            tf_listener.lookupTransform("base", "rect_goal", ros::Time(0), transform);
            tf_listener.lookupTransform("base", "tool0", ros::Time(0), base_tool);
            tf_listener.lookupTransform("tool0", "human_0/rWrist", ros::Time(0), tool_rHand);
            
            if(tool_rHand.getOrigin().length() < 1.0){ 
                HumanPose_available = true;
            }else
            {
                ROS_INFO("Please close with the robot tool!");
            }
            
            tf_available = true;
        }
        catch(tf::TransformException ex){
            continue;
            tf_available = false;
            HumanPose_available = false;
        } 

    // std::cout << JointState_available << std::endl;
    // std::cout << JointAccele_available << std::endl;
    // std::cout << HumanPose_available << std::endl;

    transformTFToKDL(transform, desired_pose);

    if(JointState_available && tf_available){
        for(int i = 0; i < 6; ++i)
        {
            q(i) = joint_angles[i];
        }

        if(ik_solver.CartToJnt(q, desired_pose, q_desired))
        {
            std::vector<double> cmd_vector;
            open_ur5_msgs::ref ref_state;
            open_ur5_msgs::cur cur_state;
            for(int i = 0; i < 6; ++i){
                ref_state.q_ref[i] = q_desired(i);
                ref_state.v_ref[i] = 0;

                cur_state.q_cur[i] = q(i);
                cur_state.v_cur[i] = joint_speed[i];

                //last_filter_vel[i] = cur_state.v_cur[i];

                if(JointAccele_available){
                    cur_state.v_cur[i] += joint_accle[i] * sample_time;

                    if(cur_state.v_cur[i] > max_speed){
                        cur_state.v_cur[i] = max_speed;
                    }
                    if(cur_state.v_cur[i] < -max_speed){
                        cur_state.v_cur[i] = -max_speed;
                    }
                    
                    
                    if(count == 1){
                        last_filter_vel[i] = cur_state.v_cur[i];
                    }else{
                        cur_state.v_cur[i] = one_order_filtering(last_filter_vel[i], cur_state.v_cur[i]);
                        last_filter_vel[i] = cur_state.v_cur[i];
                        //cout << last_filter_vel[i] << endl;
                    }
                    cmd_vector.push_back(cur_state.v_cur[i]);
                }
            }
            count++;
            ur_ref_state_pub.publish(ref_state);
            ur_cur_state_pub.publish(cur_state);

            //cout << cmd_vector.size() << endl;
            if(cmd_vector.size() == 6){
                char cmd[100];
                sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0, 0.05)", cmd_vector[0], cmd_vector[1],cmd_vector[2],cmd_vector[3],cmd_vector[4],cmd_vector[5]);
                ROS_INFO("ur script send: %s", cmd);

                if(record && fs){
                    sprintf(write_buffer, "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", 
                                    cur_state.q_cur[0], cur_state.q_cur[1], cur_state.q_cur[2], 
                                    cur_state.q_cur[3], cur_state.q_cur[4], cur_state.q_cur[5],
                                    cmd_vector[0], cmd_vector[1], cmd_vector[2],
                                    cmd_vector[3], cmd_vector[4], cmd_vector[5],
                                    joint_accle[0], joint_accle[1], joint_accle[2],
                                    joint_accle[3], joint_accle[4], joint_accle[5]);
                    // sprintf(write_buffer, "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", 
                    //                 q_desired(0), q_desired(1), q_desired(2), 
                    //                 q_desired(3), q_desired(4), q_desired(5),
                    //                 joint_angles[0], joint_angles[1], joint_angles[2],
                    //                 joint_angles[3], joint_angles[4], joint_angles[5]);
                    fs.write(write_buffer, strlen(write_buffer));
                }

                cmd_msg.data = cmd;
                ur_cmd_publisher.publish(cmd_msg);
            }

            


        }
    }
    ros::spinOnce();
    rate.sleep();
    }
    return 0;
}