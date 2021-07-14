#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <pthread.h>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <queue>
#include <ctime>
#include <vector>

#include <byp80/ShutdownGripper.h>
#include <byp80/RestartGripper.h>
#include <byp80/MoveTo.h>
#include <byp80/GetStatus.h>
#include <byp80/GetCalibrated.h>
#include <byp80/CalibrateGripper.h>


#define JOINT_NUM 6
#define PUB_RATE 20
#define TIME_STEP 1.0/PUB_RATE

#define SECURITY_DISTANCE 0.01
#define SECURITY_SPEED 0.4
#define EPISILON 0.03

using std::string;
using namespace std;
bool Joint_available;
int goal_index;
int exert_time=0;
int index_num=0;

std::vector<std::string> joint_names(JOINT_NUM);
std::vector<double> joint_angles(JOINT_NUM);
std::vector<double> joint_speed(JOINT_NUM);

void* goal_rectify_thread(void *arg)
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
            tf_listener.lookupTransform("base", "attraction_goal", ros::Time(0), desired_goal);

            tf::Vector3 goal_position = desired_goal.getOrigin();
            tf::Vector3 rectified_position = goal_position;

            rect_goal.setRotation(desired_goal.getRotation());
            rect_goal.setOrigin(desired_goal.getOrigin());
            if (goal_position.z() < 0.01){
                rectified_position.setZ(0.01);
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

bool ur_move(bool joint_available , KDL::JntArray q , KDL::Frame desired_pose , KDL::JntArray q_desired , TRAC_IK::TRAC_IK &ik_solver,double max_speed,std_msgs::String cmd_msg , ros::Publisher ur_move_publisher,double Kp,double Td)
{
	if(joint_available)
	{
		for(int i=0;i<JOINT_NUM;++i)
		{
			q(i)=joint_angles[i];
			//printf("q_current[%d]:%f\n",i,joint_angles[i]);
		}

		if(ik_solver.CartToJnt(q,desired_pose,q_desired))//ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, KDL::Twist tolerances);tolerances可有可无
		{
			std::vector<double>cmd_vector;
			for(int i=0;i<JOINT_NUM;++i)
				{
					double delta=q_desired(i)-q(i);
					double speed=Kp*delta-Td*joint_speed[i]; //PD
					if(speed>max_speed)
					{
						speed=max_speed;
					}
					if(speed<-max_speed)
					{
						speed=-max_speed;
					}
					cmd_vector.push_back(speed);
					//printf("speed[%d]:%f\n",i,speed);
				}
				char cmd[100];
				sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0, 0.05)", cmd_vector[0], cmd_vector[1], cmd_vector[2], cmd_vector[3], cmd_vector[4], cmd_vector[5]);
				ROS_INFO("ur script send:%s",cmd);

				cmd_msg.data=cmd;
				ur_move_publisher.publish(cmd_msg);
		}
	}
	return true;
}

int judgejoint(tf::StampedTransform& one_joint, tf::StampedTransform& two_joint, tf::StampedTransform& three_joint, tf::StampedTransform& four_joint, tf::StampedTransform& five_joint, tf::StampedTransform& six_joint)
{

	//double arr[5];
	double *arr = new double[6];
	int j;
	arr[0] = one_joint.getOrigin().length();
	arr[1] = two_joint.getOrigin().length();
	arr[2] = three_joint.getOrigin().length();
	arr[3] = four_joint.getOrigin().length();
	arr[4] = five_joint.getOrigin().length();
	arr[5] = six_joint.getOrigin().length();
	double min = arr[0];
	for(int i = 0; i < 5; i++)
	{
		if(min >= arr[i])
		{
			min = arr[i];
			j = i;
		}
		else
			min = min;
	}
	delete arr;
	return j;
}

bool moveTo(ros::ServiceClient &move_to_client, byp80::MoveTo &srv_move, int position, int speed, int acc, int torque, int tolerance = 100, bool waitFlag = true)
{
	srv_move.request.position = position;
	srv_move.request.speed = speed;
	srv_move.request.acceleration = acc;
	srv_move.request.torque = torque;
	srv_move.request.tolerance = tolerance;
	srv_move.request.waitFlag = waitFlag;
	move_to_client.call(srv_move);
	//cout << "通讯：" << move_to_client.call(srv_move) << endl;

	return true;
}

void jointstateCallback(const sensor_msgs::JointState& joint_state)
{
	joint_angles.clear();
	joint_speed.clear();
	std::vector<std::string> joint_names_recv=joint_state.name;
	for(auto it=joint_names.begin();it!=joint_names.end();++it)
	{
		for(auto it_recv=joint_names_recv.begin();it_recv!=joint_names_recv.end();++it_recv)
		{
			if(*it==*it_recv)
			{
				int idx=it_recv - joint_names_recv.begin();
				int i=it - joint_names_recv.begin();
				joint_angles.push_back(joint_state.position[idx]);
				joint_speed.push_back(joint_state.velocity[idx]);
				break;
			}
		}
	}
	Joint_available=true;
}

using namespace std;

void* controlGripper(void* arg){
	ros::NodeHandle nh;
	tf::TransformListener tf_listener;
	tf::StampedTransform lhz_lHand;
	tf::StampedTransform lhz_lArm;
    tf::StampedTransform xcj_lHand;
    tf::StampedTransform xcj_lArm;

	ros::ServiceClient move_to_client = nh.serviceClient<byp80::MoveTo>("move_to");
	byp80::MoveTo srv_move;
	moveTo(move_to_client, srv_move, 80, 100,50,1,100,true);
	moveTo(move_to_client, srv_move, 0, 100, 50, 1, 100, true);
    ros::Rate rate(30);

	while(ros::ok()){
		try
		{
			tf_listener.waitForTransform("base", "lhz/lWrist", ros::Time(0), ros::Duration(3.0));
			tf_listener.waitForTransform("base", "lhz/lArm", ros::Time(0), ros::Duration(3.0));

			tf_listener.lookupTransform("base", "lhz/lWrist", ros::Time(0), lhz_lHand);
			tf_listener.lookupTransform("base", "lhz/lArm", ros::Time(0), lhz_lArm);
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			continue;
		}

        // try{
        //     tf_listener.waitForTransform("base", "xcj/lWrist", ros::Time(0), ros::Duration(3.0));
        //     tf_listener.waitForTransform("base", "xcj/lArm", ros::Time(0), ros::Duration(3.0));
        //
        //     tf_listener.lookupTransform("base", "xcj/lWrist", ros::Time(0), xcj_lHand);
        //     tf_listener.lookupTransform("base", "xcj/lArm", ros::Time(0), xcj_lArm);
        // }
        // catch(const std::exception& e)
		// {
		// 	std::cerr << e.what() << '\n';
		// }

        //if(lhz_lHand.getOrigin().length() < xcj_lHand.getOrigin().length()){
            if(lhz_lHand.getOrigin().z() > lhz_lArm.getOrigin().z()){
    			moveTo(move_to_client, srv_move, 0, 100, 50, 1, 100, true);
    		}
    		//if(lHand.getOrigin().z() <= lArm.getOrigin().z())
            else{
    			moveTo(move_to_client, srv_move, 80, 100,50,1,100,true);
    		}
        // }else{
        //     if(xcj_lHand.getOrigin().z() > xcj_lArm.getOrigin().z()){
    	// 		moveTo(move_to_client, srv_move, 0, 100, 50, 1, 100, true);
    	// 	}
    	// 	//if(lHand.getOrigin().z() <= lArm.getOrigin().z())
        //     else{
    	// 		moveTo(move_to_client, srv_move, 80, 100,50,1,100,true);
    	// 	}
        // }


        ros::spinOnce();
        rate.sleep();
	}
}

int main(int argc,char** argv)
{
	joint_names.push_back("shoulder_pan_joint");
	joint_names.push_back("shoulder_lift_joint");
	joint_names.push_back("elbow_joint");
	joint_names.push_back("wrist_1_joint");
	joint_names.push_back("wrist_2_joint");
	joint_names.push_back("wrist_3_joint");

	std_msgs::String cmd_msg;
	double Kp=2.2;
	double Td=0.2;
	double max_speed=0.4;
	double mid_speed=0.3;

	double human_space_radius=0.85;
	double robot_space_radius=0.75;
	double base_radius=0.3;

	ros::init(argc,argv,"ur_move_cmd_hand");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	pthread_t rectify_thread[2];
	pthread_create(&rectify_thread[0], NULL, goal_rectify_thread, NULL);
	//pthread_create(&rectify_thread[1], NULL, controlGripper, NULL);


	ros::Publisher ur_move_publisher=nh.advertise<std_msgs::String>("/ur_driver/URScript",1);
	
	ros::Subscriber joint_state_sub=nh.subscribe("/joint_states",1,jointstateCallback);

	tf::TransformBroadcaster br;
	tf::TransformListener tf_listener;


	string urdf_param="/robot_description";
	string base="base";
	string tip="tool0";
	TRAC_IK::TRAC_IK ik_solver(base,tip,urdf_param,0.005,1e-5,TRAC_IK::Distance);

	KDL::JntArray q(6);
	KDL::JntArray q_desired(6);
	KDL::Frame desired_pose;

	//tf::StampedTransform desired_goal;
	tf::StampedTransform base_tool;
	tf::StampedTransform tool_rHand;
	tf::StampedTransform home;
	tf::StampedTransform rHand_order;
    tf::StampedTransform xcj_rHand;
	tf::StampedTransform attraction;
	tf::StampedTransform desired_goal;

	tf::Transform rHand_position;
	tf::Transform desired_position;
	tf::Transform go_home;
	//tf::Transform rHand;
	go_home.setOrigin(tf::Vector3(0,-0.42,0.3));


	tf::Quaternion qq(0.724, 0.689, -0.018, -0.026);
	qq=qq.normalize();
	//qq.setRPY(0,0,0);
	go_home.setRotation(qq);
	//rHand.setOrigin(tf::Vector3(0,-0.4,0.3));
	//rHand.setRotation(qq);
	//tf::StampedTransform base_tool;

	ros::spinOnce();
	ros::Rate rate(60);
	std::cout << "controlling using time step at: " << TIME_STEP << std::endl;
	while(ros::ok())
	{

		while(ros::ok())
		{
			cout << "Waiting connection!" << endl;
			try
			{
				tf_listener.waitForTransform("base","tool0",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base","tool0",ros::Time(0),base_tool);

				tf_listener.waitForTransform("tool0","human_0/rHand",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("tool0","human_0/rHand",ros::Time(0),tool_rHand);

				tf_listener.waitForTransform("base","human_0/rHand",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base","human_0/rHand",ros::Time(0),rHand_order);

				if(sqrt((tool_rHand.getOrigin().x())*(tool_rHand.getOrigin().x()) + (tool_rHand.getOrigin().y())*(tool_rHand.getOrigin().y()) + (tool_rHand.getOrigin().z())*(tool_rHand.getOrigin().z()))<0.3);
				{
					cout << "connected successfully!" << endl;
					break;
				}
			}
			catch(tf::TransformException ex){
				ROS_ERROR(ex.what());
				continue;
			}
		}


		while(ros::ok())
		{
			//sleep(3);
			br.sendTransform(tf::StampedTransform(go_home,ros::Time::now(),"base","home"));
			try
			{
				tf_listener.waitForTransform("base","home",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base","home",ros::Time(0),home);

				tf_listener.waitForTransform("base","tool0",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base","tool0",ros::Time(0),base_tool);
				transformTFToKDL(home, desired_pose);
			}
			catch(tf::TransformException ex)
			{
				ROS_ERROR(ex.what());
				continue;
			}
			ur_move(Joint_available , q , desired_pose , q_desired ,ik_solver, max_speed, cmd_msg , ur_move_publisher,Kp,Td);
			if(fabs(base_tool.getOrigin().x()-home.getOrigin().x()) < 0.03 && fabs(base_tool.getOrigin().y() -home.getOrigin().y() ) < 0.03 && fabs(base_tool.getOrigin().z() - home.getOrigin().z()) < 0.03)
			{
				ROS_INFO("go home,please ready!");
				sleep(3);
				break;
			}

			ros::spinOnce();
			rate.sleep();

			//break;
		}

        while(ros::ok()){
            try
			{
				tf_listener.waitForTransform("base","tool0",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base","tool0",ros::Time(0),base_tool);

                tf_listener.waitForTransform("base","human_0/rHand",ros::Time(0),ros::Duration(3.0));
                tf_listener.lookupTransform("base","human_0/rHand",ros::Time(0),rHand_order);
                break;
			}
			catch(tf::TransformException ex)
			{
				ROS_ERROR(ex.what());
				continue;
			}
        }

        tf::Vector3 offset(base_tool.getOrigin().x()-rHand_order.getOrigin().x(),
                        base_tool.getOrigin().y()-rHand_order.getOrigin().y(),
                        base_tool.getOrigin().z()-rHand_order.getOrigin().z());

		while(ros::ok())
		{

			try{
				//tf_listener.waitForTransform("base", "human_1/rHand", ros::Time(0), ros::Duration(3.0));
				tf_listener.lookupTransform("base", "human_0/rHand", ros::Time(0), rHand_order);
                desired_position.setOrigin(tf::Vector3(rHand_order.getOrigin().x()+offset.x(),
                                                       rHand_order.getOrigin().y()+offset.y(),
                                                       rHand_order.getOrigin().z()+offset.z()));
				desired_position.setRotation(qq);
			}
			catch(tf::TransformException ex)
			{
				ROS_ERROR(ex.what());
				continue;
			}

            // try{
            //     tf_listener.lookupTransform("base", "xcj/rWrist", ros::Time(0), xcj_rHand);
            //     if(rHand_order.getOrigin().length() < xcj_rHand.getOrigin().length()){
            //         desired_position.setOrigin(tf::Vector3(rHand_order.getOrigin().x()+offset.x(),
            //                                                rHand_order.getOrigin().y()+offset.y(),
            //                                                rHand_order.getOrigin().z()+offset.z()));

            //     }else if(rHand_order.getOrigin().length() > xcj_rHand.getOrigin().length()){
            //         desired_position.setOrigin(tf::Vector3(xcj_rHand.getOrigin().x()+offset.x(),
            //                                                xcj_rHand.getOrigin().y()+offset.y(),
            //                                                xcj_rHand.getOrigin().z()+offset.z()));
            //     }

			// 	desired_position.setRotation(qq);

            // }catch(tf::TransformException ex){
            //     ROS_ERROR(ex.what());
            //     //continue;
            // }

			br.sendTransform(tf::StampedTransform(desired_position,ros::Time::now(),"base","attraction_goal"));
			try{
				tf_listener.waitForTransform("base","attraction_goal",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base", "attraction_goal", ros::Time(0), attraction);

				tf_listener.waitForTransform("base","tool0",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base","tool0",ros::Time(0),base_tool);

				tf_listener.waitForTransform("base","rect_goal",ros::Time(0),ros::Duration(3.0));
				tf_listener.lookupTransform("base","rect_goal",ros::Time(0),desired_goal);
				transformTFToKDL(desired_goal, desired_pose);
			}
			catch(tf::TransformException ex)
			{
				ROS_ERROR(ex.what());
				continue;
			}
			ur_move(Joint_available , q ,desired_pose , q_desired ,ik_solver, max_speed, cmd_msg , ur_move_publisher,Kp,Td);

            tf::Vector3  tool_attraction(attraction.getOrigin().x()-base_tool.getOrigin().x(),
    						     		 attraction.getOrigin().y()-base_tool.getOrigin().y(),
    							 		 attraction.getOrigin().z()-base_tool.getOrigin().z());

			if(sqrt((tool_attraction.x())*(tool_attraction.x()) + (tool_attraction.y())*(tool_attraction.y()) + (tool_attraction.z())*(tool_attraction.z()))>0.8)
			{
				ROS_INFO("Human get outoff our workspace");
				break;
			}
			ros::spinOnce();
			rate.sleep();
		}

		try
		{
			tf_listener.waitForTransform("base","human_0/rHand",ros::Time(0),ros::Duration(3.0));
			tf_listener.lookupTransform("base","human_0/rHand",ros::Time(0),rHand_order);

		}
		catch(tf::TransformException ex)
		{
			ROS_INFO("No human here");;
			break;
		}

	}

return 0;
}