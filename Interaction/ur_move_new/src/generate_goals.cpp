#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_msgs/String.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <queue>
#include <vector>
#include <ur_move_new/gripper.h>
#define SECURITY_DISTANCE 0.4
#define SECURITY_SPEED 0.4
#define EPISILON 0.03
using std::queue;
using std::vector;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "generate_goal");
	ros::NodeHandle nh;
	tf::TransformListener tf_linstener;
    tf::TransformBroadcaster br;
	tf::StampedTransform base_tool;

	vector<tf::Transform> att_goals;
	tf::Transform goal1, goal2, goal3, goal4, goal5, goal6;
	tf::Transform place_goal;

	goal1.setOrigin(tf::Vector3(-0.68, -0.16, -0.13));
    goal2.setOrigin(tf::Vector3(-0.68, -0.10, -0.13));
    goal3.setOrigin(tf::Vector3(-0.68, -0.05, -0.13));
    goal4.setOrigin(tf::Vector3(-0.68, 0.02, -0.13));
    goal5.setOrigin(tf::Vector3(-0.68, 0.08, -0.13));
    goal6.setOrigin(tf::Vector3(-0.55, -0.08, 0.310));
    place_goal.setOrigin(tf::Vector3(0.109, -0.505, 0.310));

	tf::Quaternion q1(0.0036, 0.999, -0.006, -0.0021);
    tf::Quaternion q2(0.7049, 0.7092, -0.00076, -0.00339);
    tf::Quaternion q3(-0.046, 0.997, -0.0612, -0.00439);
	q1 = q1.normalize();
    q2 = q2.normalize();
    q3 = q3.normalize();
	//q.setRPY(0.0, 0.0, 0.0);
	goal1.setRotation(q1);
	goal2.setRotation(q1);
	goal3.setRotation(q1);
   	goal4.setRotation(q1);
	goal5.setRotation(q1);
    goal6.setRotation(q2);
    place_goal.setRotation(q3);
  
    att_goals.push_back(place_goal);
	att_goals.push_back(goal6);
    att_goals.push_back(goal1);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
	att_goals.push_back(goal6);
    att_goals.push_back(goal2);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
    att_goals.push_back(goal6);
	att_goals.push_back(goal3);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
    att_goals.push_back(goal6);
    att_goals.push_back(goal4);
    att_goals.push_back(goal6);
    att_goals.push_back(place_goal);
    att_goals.push_back(goal6);
	att_goals.push_back(goal5);
    att_goals.push_back(goal6);
	queue<int> target_queue;
    target_queue.push(0);
    target_queue.push(1);
	target_queue.push(2);
	target_queue.push(3);
    target_queue.push(4);
    target_queue.push(5);
	target_queue.push(6);
	target_queue.push(7);
    target_queue.push(8);
	target_queue.push(9);
	target_queue.push(10);
	target_queue.push(11);
	target_queue.push(12);
    target_queue.push(13);
	target_queue.push(14);
    target_queue.push(15);
	target_queue.push(16);
	target_queue.push(17);
    target_queue.push(18);
	target_queue.push(19);
	int goal_index = target_queue.front();

	target_queue.pop();
	target_queue.push(goal_index);

    Gripper gripper;
	ros::Rate rate(30);
	tf::StampedTransform neck_tool, marker_base;
	bool dis_op = false;
	while(ros::ok())
	{
		// try{
		// 	tf_linstener.waitForTransform("base", "correct_1", ros::Time(0), ros::Duration(3.0));//need
		// 	tf_linstener.lookupTransform("base", "correct_1", ros::Time(0), marker_base);//need
		// }
		// catch(tf::TransformException ex){
		// 	continue;
		// }
		// tf::Transform marker;
		// marker.setOrigin(marker_base.getOrigin());
		// marker.setRotation(marker_base.getRotation());
		br.sendTransform(tf::StampedTransform(att_goals[goal_index], ros::Time::now(), "base", "desired_goal"));
		//br.sendTransform(tf::StampedTransform(marker, ros::Time::now(), "base", "desired_goal"));
		std::cout << "goal_index: " << goal_index << std::endl;
		try{
			tf_linstener.waitForTransform("base", "tool0", ros::Time(0), ros::Duration(3.0));//need
			tf_linstener.lookupTransform("base", "tool0", ros::Time(0), base_tool);//need
			

			tf_linstener.waitForTransform("tool0", "human_0/neck", ros::Time(0), ros::Duration(3.0));//need
			tf_linstener.lookupTransform("tool0", "human_0/neck", ros::Time(0), neck_tool);//need

			if(neck_tool.getOrigin().length() <= 1.0){
				dis_op = true;
			}
			else{
				dis_op = false;
				//continue;
			}
		}
		catch(tf::TransformException ex){
			continue;
		}



		if(fabs(base_tool.getOrigin().x()-att_goals[goal_index].getOrigin().x()) < 0.03 && fabs(base_tool.getOrigin().y() -att_goals[goal_index].getOrigin().y() ) < 0.03 && fabs(base_tool.getOrigin().z() - att_goals[goal_index].getOrigin().z()) < 0.03)
		{
            if(goal_index == 2 || goal_index == 6 || goal_index == 10 || goal_index == 14 || goal_index == 18){
                gripper.clamp();
                std::cout << goal_index << std::endl;
				goal_index = target_queue.front();
				target_queue.pop();
				target_queue.push(goal_index);
            }else if (goal_index == 0 || goal_index == 4 || goal_index == 8 || goal_index == 12 || goal_index == 16){
				if(dis_op)
				{
					gripper.loose();
                	std::cout << goal_index << std::endl;
					goal_index = target_queue.front();
					target_queue.pop();
					target_queue.push(goal_index);
				}else{
					//goal_index = goal_index;
					std::cout << "No people.." << std::endl;
					}
					
			}else{
				goal_index = target_queue.front();
				target_queue.pop();
				target_queue.push(goal_index);
			}
		}
        ros::spinOnce();
		//rate.sleep();
	}
	return 0;
}