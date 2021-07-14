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



#include <iostream>
#define JOINT_NUM 6
#define PUB_RATE 20
#define TIME_STEP 1.0/PUB_RATE


using std::string;
bool JointState_available;

std::vector<std::string> joint_names(JOINT_NUM);
std::vector<double> joint_angles(JOINT_NUM);
std::vector<double> joint_speed(JOINT_NUM);

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

void* goal_rectify__thread(void *arg)
{
    tf::TransformBroadcaster br;
    tf::Transform rect_goal;
    tf::TransformListener tf_listener;
    tf::StampedTransform desired_goal;

    double work_space_radius = 0.90;
    double base_radius = 0.2;

    ros::Rate rate(30);
    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base", "desired_goal", ros::Time(0), desired_goal);

            tf::Vector3 goal_position = desired_goal.getOrigin();
            tf::Vector3 rectified_position = goal_position;

            rect_goal.setRotation(desired_goal.getRotation());
            rect_goal.setOrigin(desired_goal.getOrigin());
            if (goal_position.z() < -0.15){
                rectified_position.setZ(-0.15);
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
             
    std_msgs::String cmd_msg;
    double Kp = 2.5;
    double Td = 0.2;
    double max_speed = 0.5;

    std::string record_dir = "/home/xuchengjun/open_ros_codegen/data/guiji.txt";
    //std::string record_dir_2 = "/home/xuchengjun/open_ros_codegen/data/ee_track.txt";
    bool record = false;
    char* write_buffer = NULL;
    std::ofstream fs;
    if(record){
        fs.open(record_dir.c_str());
        if(fs){
            //fs << "x \n";
        }
        write_buffer = (char*)malloc(256);
    }

    ros::init(argc, argv, "ur_move_cmd");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

    //pthread_t rectify_thread;
    //pthread_create(&rectify_thread, NULL, goal_rectify__thread, NULL);

    ros::Publisher ur_cmd_publisher = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);
    ros::Publisher ur_joint_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);

    string urdf_param = "/robot_description";

    string base = "base";
    string tip = "tool0";

    TRAC_IK::TRAC_IK ik_solver(base, tip, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);

    KDL::JntArray q(6);
    KDL::JntArray q_desired(6);
    KDL::Frame desired_pose;
    tf::StampedTransform transform;
    tf::StampedTransform control_speed;
    tf::StampedTransform base_tool;
    bool tf_available = false;

    ros::spinOnce();
    ros::Rate rate(30);
    std::cout << "controlling using time step at: " << TIME_STEP << std::endl;

    while(ros::ok())
    {
        try{
            tf_listener.lookupTransform("base", "desired_goal", ros::Time(0), transform);
            //tf_listener.lookupTransform("base", "human_0/rWrist", ros::Time(0), control_speed);
            tf_listener.lookupTransform("base", "tool0", ros::Time(0), base_tool);
            transformTFToKDL(transform, desired_pose);
			//std::cout << "0" << std::endl;
            tf_available = true;
           //  std::cout << "1" << std::endl;
        }
        catch(tf::TransformException ex){
            continue;
            std::cout << "2 "<< std::endl;
            tf_available = false;
            
        } 

        if(record && fs){
            sprintf(write_buffer, "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", 
                    q_desired(0), q_desired(1), q_desired(2), 
                    q(0),q(1),q(2));
            fs.write(write_buffer, strlen(write_buffer));
        }

        // if(fabs(control_speed.getOrigin().x()) >= 1.5){
        // 	double Kp = 2.2;
        // 	max_speed = 1.0;
        // }
        // else if ( 0.2 <= fabs(control_speed.getOrigin().x()) < 1.5){
        // 	double Kp = 1.0;
        // 	max_speed = 0.5;
        // }
        // else if(fabs(control_speed.getOrigin().x()) < 0.4 ){
        // 	double Kp = 0.5;
        // }

        std::cout << JointState_available<< std::endl;

        if(JointState_available && tf_available){

            for(int i = 0; i < 6; ++i)
                {
                    q(i) = joint_angles[i];
                    printf("q_current[%d]:%f\n",i,joint_angles[i]);
                }
            if(ik_solver.CartToJnt(q, desired_pose, q_desired))
            {
                std::vector<double> cmd_vector;

                for(int i = 0; i < 6; ++i)
                {
                    double delta = q_desired(i) - q(i);
                    double speed = Kp*delta - Td*joint_speed[i];
                    if(speed > max_speed){
                        speed = max_speed;
                    }
                    if(speed < -max_speed){
                        speed = -max_speed;
                    }
                    cmd_vector.push_back(speed);
                }

               // ur_joint_publisher.publish(joint_publish);

                char cmd[100];
                sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0, 0.05)", cmd_vector[0], cmd_vector[1], cmd_vector[2], cmd_vector[3], cmd_vector[4], cmd_vector[5]);
                ROS_INFO("ur script send: %s", cmd);
  

  
                cmd_msg.data = cmd;
                ur_cmd_publisher.publish(cmd_msg);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}