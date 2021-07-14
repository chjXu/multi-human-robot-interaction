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
// //------------爪子---------------------//
// #include <byp80/ByStatus.h>
// #include <byp80/ShutdownGripper.h>
// #include <byp80/RestartGripper.h>
// #include <byp80/MoveTo.h>
// #include <byp80/GetStatus.h>
// #include <byp80/GetCalibrated.h>
// #include <byp80/CalibrateGripper.h>

#include <pthread.h>
#include <cmath>

#include <iostream>


#define JOINT_NUM 6
#define PUB_RATE 20
#define TIME_STEP 1.0/PUB_RATE


using std::string;
bool JointState_available;


// Eigen::Matrix<double,3,3> rot_origin;
// Eigen::Matrix<double,3,3> rot;
// Eigen::Matrix<double,3,3> yy;   //绕y轴旋转
// // yy << -1.,0.,0.,0.,1.,0.,0.,0.,-1.;
// tf::Transform transform_new;

// double x , y ,z;

std::vector<std::string> joint_names(JOINT_NUM);  //机器人6个关节的名称列表
std::vector<double> joint_angles(JOINT_NUM);      //机器人6个关节的关节角度
std::vector<double> joint_speed(JOINT_NUM);       //机器人6个关节的关节速度

void jointStateCallback(const sensor_msgs::JointState& joint_state)
{
    joint_angles.clear();  //关节角度
    joint_speed.clear();   //关节速度
    std::vector<std::string> joint_names_recv = joint_state.name;  //关节名称  .name是一个列表
    for(auto it = joint_names.begin(); it !=joint_names.end(); ++it)
    {
        for(auto it_recv = joint_names_recv.begin(); it_recv != joint_names_recv.end(); ++it_recv)
        {
            if (*it_recv == *it)
            {
                int idx = it_recv - joint_names_recv.begin();
                int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);  //关节角度 ，其微分就是角速度 ， 即下面的joint_speed
                joint_speed.push_back(joint_state.velocity[idx]);   //关节角速度
                break;
            }
        }
    }
    JointState_available = true;
}

/*void* generate_thread(void *arg)
{
    ros::NodeHandle* nh = (ros::NodeHandle*) arg;
    tf::TransformBroadcaster br;
    tf::Transform base_goal;    
    while(ros::ok())
    {
    base_goal.setOrigin(tf::Vector3(0.65, 0.0, 0.0));
    tf::Quaternion qq;
    qq.setRPY(0,0,-M_PI/2);
    base_goal.setRotation(qq);
    br.sendTransform(tf::StampedTransform(base_goal, ros::Time::now(), "vicon/marker_0/marker_0", "base"));
    }
}
/*void* goal_generate_thread(void *arg)
{
    ros::NodeHandle* nh = (ros::NodeHandle*) arg;
    tf::TransformBroadcaster br;
    tf::Transform goal;
    goal.setOrigin(tf::Vector3(0.0,0.0,0.1));
    tf::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    goal.setRotation(q);
    std::string goal_frame;
    ros::Rate rate(30);
    while(ros::ok()){
        nh->param<std::string>("goal_frame", goal_frame, "marker_10");

        br.sendTransform(tf::StampedTransform(goal, ros::Time::now(), goal_frame.c_str(), "desired_goal"));
        rate.sleep();
    }
}
*/
void* goal_rectify__thread(void *arg)
{
    tf::TransformBroadcaster br;
    tf::Transform rect_goal;
    tf::TransformListener tf_listener;
    tf::StampedTransform desired_goal;

    double work_space_radius = 0.85;   //工作空间
    double base_radius = 0.2;  //

    ros::Rate rate(60);
    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base", "desired_goal", ros::Time(0), desired_goal);

            tf::Vector3 goal_position = desired_goal.getOrigin();
            tf::Vector3 rectified_position = goal_position;  //对姿态进行修正

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
    //关节名称
	joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");
    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");

  //  bool publish_goal = true;
/*    if(argc > 1)
    {
        std::string arg;
        arg = argv[1];
        if(arg == "false"){
            publish_goal = false;
            ROS_INFO("Publishing goal disabled");
        }
    }
*/                
    std_msgs::String cmd_msg;  //ros中的标准msgs信息,这个是要发送给 /ur_driver/URScript 的
    double Kp = 2.2;
    double Td = 0.2;
    double max_speed = 1.5;   //最大速度

    ros::init(argc, argv, "ur_move_cmd");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
 //   pthread_t start_base;
  //  pthread_create(&start_base, NULL, generate_thread, (void*)&nh);
 //   if(publish_goal){
 //       pthread_t goal_thread;
 //       pthread_create(&goal_thread, NULL, goal_generate_thread, (void*)&nh);
  //  }
    pthread_t rectify_thread;  //pthread_t型的变量:原型是--　typedef unsigned long int pthread_t;是多线程的标识符
    pthread_create(&rectify_thread, NULL, goal_rectify__thread, NULL);  //函数pthread_create用来创建一个线程

    ros::Publisher ur_cmd_publisher = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);   // /ur_driver/URScript：话题名称，类型是std_msgs::String，允许我们向该话题发布URScript脚本命令
    ros::Publisher ur_joint_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 1); // /joint_states：话题名称，类型是：sensor_msgs::JointState，实现机械臂的运动，就是向话题joint_states添加有关关节的消息

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);  //

    string urdf_param = "/robot_description"; //机器人的模型,这个必须要添加到求解器中

    string base = "base";
    string tip = "tool0";

    //TRAC-IK和Orocos KDL类似，也是一种基于数值解的机器人运动学求解器，但是在算法层面上进行了很多改进
    //TRAC_IK::TRAC_IK ik_solver(string base_link, string tip_link, string URDF_param="/robot_description", double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed); 
    TRAC_IK::TRAC_IK ik_solver(base, tip, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);  

    KDL::JntArray q(6);
    KDL::JntArray q_desired(6);
    KDL::Frame desired_pose;
    tf::StampedTransform transform,transform_marker_0;
    tf::StampedTransform control_speed;
    bool tf_available = false;

    ros::spinOnce();    //回调关节状态，得到“上一时刻”的关节角度位置以及关节角速度，作为上一时刻的信息
    ros::Rate rate(30); 
    std::cout << "controlling using time step at: " << TIME_STEP << std::endl;

    while(ros::ok())
    {
        try{
            tf_listener.lookupTransform("base", "new", ros::Time(0), transform);  //监听你发送的tf信息
            // tf_listener.lookupTransform("marker_0", "new", ros::Time(0), transform_marker_0);
            //tf_listener.lookupTransform("base", "human_1/rHand", ros::Time(0), control_speed);

            //Quaternion
            // W = transform.getRotation().getW();
            // X = transform.getRotation().getX();
            // Y = transform.getRotation().getY();
            // Z = transform.getRotation().getZ();

            // //trans
            // x = transform.getOrigin().x();
            // y = transform.getOrigin().y();
            // z = transform.getOrigin().z();

            // Eigen::Quaterniond q(W,X,Y,Z);  //得到四元数
            // q.normalize();  //自身规范化
            // rot_origin = q.toRotationMatrix();

            // rot = yy * ros_origin;
            // q_new = rot;
            // transform_new.setOrigin(tf::Vector3(x,y,z));
            // transform_new.setRotation(q);
            

            // x = transform_marker_0.getOrigin().x();
            // x += 0.032; 
            // y = transform_marker_0.getOrigin().y();
            // // y += 0.05;
            // z = transform_marker_0.getOrigin().z();

            // auto rotation = transform.getRotation();  //四元数

            // transform_new.setOrigin(tf::Vector3(x,y,z));
            // transform_new.setRotation(rotation);


            transformTFToKDL(transform, desired_pose);   //transformTFToKDL：运动学/动力学库，将监听得到的tf信息转换到KDL中
            // transformTFToKDL(transform_new, desired_pose);
			//std::cout << "0" << std::endl;
            tf_available = true;
           //  std::cout << "1" << std::endl;
        }
        catch(tf::TransformException ex){
            continue;
            std::cout << "2 "<< std::endl;
            tf_available = false;
            
        } 
       // if(fabs(control_speed.getOrigin().x()) >= 1.5){
        //	double Kp = 2.2;
        //	max_speed = 1.5;
       // }
       // else if ( 0.2 <= fabs(control_speed.getOrigin().x()) < 1.5){
        //	double Kp = 1.0;
        //	max_speed = 0.5;
        //}
        //else if(fabs(control_speed.getOrigin().x()) < 0.4 ){
        //	double Kp = 0.5;
        //}
 //       if(!tf_available)
 //       {
  //      	    sensor_msgs::JointState joint_publish;
  //              joint_publish.header.stamp = ros::Time::now();
  //              joint_publish.position = {-1.2472029006273857, -1.8607643190573073, -1.6143243547091355, -1.1299156877085212, -4.6393170504734424, 2.1053455841321425};
               /* std::vector<std_msgs::String> joint_name_std(6);

                for(int i = 0; i < 6; ++i)
                {
                	joint_name_std.at(i).data = joint_names[i];
                	joint_publish.position.push_back(origin_pose[i]);
                }*/
                //joint_publish.name = joint_name_std;
  //              joint_publish.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
   //             ur_joint_publisher.publish(joint_publish);
  //              continue;
   //     }


        std::cout << JointState_available<< std::endl;

        //关节状态获取得到   tf获取得到
        if(JointState_available && tf_available){

            for(int i = 0; i < 6; ++i)
                {
                    q(i) = joint_angles[i]; //q是joint_angles（相当于上一时刻）
                    printf("q_current[%d]:%f\n",i,joint_angles[i]);
                }
            //int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, 
            //                             KDL::JntArray& return_joints, KDL::Twist tolerances);
            if(ik_solver.CartToJnt(q, desired_pose, q_desired))    //返回期望的q_desired，逆运动学求解器
            {
                std::vector<double> cmd_vector;

              //  sensor_msgs::JointState joint_publish;
               // joint_publish.header.stamp = ros::Time::now();
               // joint_publish.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

                for(int i = 0; i < 6; ++i)
                {
    
                	//joint_publish.position.push_back(q_desired(i));
                 //   printf("q_desired[%d]:%f\n",i,q_desired(i));
                    double delta = q_desired(i) - q(i);  //误差（q_desired:系统输入，q：上一时刻的“输出”）
                    double speed = Kp*delta + Td*joint_speed[i]; //PD控制

                    //速度判断
                    if(speed > max_speed){
                        speed = max_speed;
                    }
                    if(speed < -max_speed){
                        speed = -max_speed;
                    }

                    cmd_vector.push_back(speed);
                    printf("speed[%d]:%f\n",i,speed);   //速度控制
                }

               // ur_joint_publisher.publish(joint_publish);

                char cmd[100];  //command
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
