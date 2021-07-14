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
	ros::init(argc, argv, "collaborative_goal");
	ros::NodeHandle nh;
	tf::TransformListener tf_linstener;
    tf::TransformBroadcaster br;
	tf::StampedTransform base_tool;

	vector<tf::Transform> att_goals;
	tf::Transform ready_goal, marker_goal, ready_goal_1, middle_goal;
	tf::Transform init_goal,place_goal;

	marker_goal.setOrigin(tf::Vector3(0.109, -0.505, 0.310));
	ready_goal_1.setOrigin(tf::Vector3(0.106, -0.508, 0.337));
	ready_goal.setOrigin(tf::Vector3(0.106, -0.508, 0.437));
	middle_goal.setOrigin(tf::Vector3(-0.254, -0.483, 0.345));
	init_goal.setOrigin(tf::Vector3(-0.62, -0.102, 0.283));
    place_goal.setOrigin(tf::Vector3(-0.62, -0.102, -0.0117));	//手动赋值

	tf::Quaternion q1(0.0366, 0.715, -0.697, -0.0291);	//抓点
    tf::Quaternion q2(0.713, 0.700, 0.00382, -0.0131);	//初始点
    tf::Quaternion q3(0.714,0.699, -0.011, -0.0274);	//放下
	tf::Quaternion q4(-0.0208,0.999, 0.0206, -0.0316);	//放下
	tf::Quaternion q5(-0.0208,0.999, 0.0206, -0.0316);	//middle_point
	q1 = q1.normalize();
    q2 = q2.normalize();
    q3 = q3.normalize();
	q4 = q4.normalize();
	q5 = q5.normalize();
	//q.setRPY(0.0, 0.0, 0.0);
	marker_goal.setRotation(q1);
	ready_goal.setRotation(q1);
	init_goal.setRotation(q2);
    place_goal.setRotation(q3);
	ready_goal_1.setRotation(q4);
	middle_goal.setRotation(q5);
  
	att_goals.push_back(ready_goal_1);
    att_goals.push_back(ready_goal);	//0
	att_goals.push_back(marker_goal);	//1   //1
	//att_goals.push_back(ready_goal);
	att_goals.push_back(ready_goal_1);
	att_goals.push_back(middle_goal);
    att_goals.push_back(init_goal);
    att_goals.push_back(place_goal);
    att_goals.push_back(init_goal);
	att_goals.push_back(middle_goal);
	att_goals.push_back(ready_goal_1);


	att_goals.push_back(ready_goal);	//6
    att_goals.push_back(marker_goal); 	//2   //7
	//att_goals.push_back(ready_goal);
	att_goals.push_back(ready_goal_1);
	att_goals.push_back(middle_goal);
    att_goals.push_back(init_goal);
    att_goals.push_back(place_goal);
    att_goals.push_back(init_goal);
	att_goals.push_back(middle_goal);
	att_goals.push_back(ready_goal_1);

	att_goals.push_back(ready_goal);	//12
    att_goals.push_back(marker_goal);	//3		//13
	//att_goals.push_back(ready_goal);
	att_goals.push_back(ready_goal_1);
	att_goals.push_back(middle_goal);
	att_goals.push_back(init_goal);
    att_goals.push_back(place_goal);
    att_goals.push_back(init_goal);
	att_goals.push_back(middle_goal);
	att_goals.push_back(ready_goal_1);
	
	att_goals.push_back(ready_goal);
    att_goals.push_back(marker_goal);	//4		//19
	//att_goals.push_back(ready_goal);
	att_goals.push_back(ready_goal_1);
	att_goals.push_back(middle_goal);
	att_goals.push_back(init_goal);
    att_goals.push_back(place_goal);
    att_goals.push_back(init_goal);
	att_goals.push_back(middle_goal);
	att_goals.push_back(ready_goal_1);

	att_goals.push_back(ready_goal);
    att_goals.push_back(marker_goal);	//5		//25
	//att_goals.push_back(ready_goal);
	att_goals.push_back(ready_goal_1);
	att_goals.push_back(middle_goal);
	att_goals.push_back(init_goal);
    att_goals.push_back(place_goal);
    att_goals.push_back(init_goal);
	att_goals.push_back(middle_goal);
	att_goals.push_back(ready_goal_1);
 
	queue<int> target_queue;
	for(int i=0; i<att_goals.size(); ++i){
		target_queue.push(i);
	}
    
	int goal_index = target_queue.front();

	target_queue.pop();
	target_queue.push(goal_index);

    Gripper gripper;
	ros::Rate rate(30);
	tf::StampedTransform neck_tool, marker_base;
	bool dis_op = false;
	int i=1;
	while(ros::ok())
	{
		br.sendTransform(tf::StampedTransform(att_goals[goal_index], ros::Time::now(), "base", "desired_goal"));
		//br.sendTransform(tf::StampedTransform(marker, ros::Time::now(), "base", "desired_goal"));
		std::cout << "goal_index: " << goal_index << std::endl;
		try{
			tf_linstener.waitForTransform("base", "tool0", ros::Time(0), ros::Duration(3.0));//need
			tf_linstener.lookupTransform("base", "tool0", ros::Time(0), base_tool);//need
		}
		catch(tf::TransformException ex){
			continue;
		}

		if(fabs(base_tool.getOrigin().x()-att_goals[goal_index].getOrigin().x()) < 0.03 && fabs(base_tool.getOrigin().y() -att_goals[goal_index].getOrigin().y() ) < 0.03 && fabs(base_tool.getOrigin().z() - att_goals[goal_index].getOrigin().z()) < 0.03)
		{
            if(goal_index == 6 || goal_index == 15 || goal_index == 24 || goal_index == 33 || goal_index == 42){
                gripper.loose();
				
                std::cout << goal_index << std::endl;
				goal_index = target_queue.front();
				target_queue.pop();
				target_queue.push(goal_index);
            }else if (goal_index == 1 || goal_index == 10 || goal_index == 19 || goal_index == 28 || goal_index == 37){
				while (ros::ok())
				{
					try{
						tf_linstener.waitForTransform("base", "correct_" + std::to_string(i), ros::Time(0), ros::Duration(3.0));//need
						tf_linstener.lookupTransform("base", "correct_" + std::to_string(i), ros::Time(0), marker_base);//need
						break;
					}
					catch(tf::TransformException ex){
						continue;
					}
				}

				goal_index = target_queue.front();
				att_goals[goal_index].setOrigin(marker_base.getOrigin());
				att_goals[goal_index].setRotation(marker_base.getRotation());
				
                std::cout << goal_index << std::endl;

				target_queue.pop();
				target_queue.push(goal_index);
				++i;
	
					
			}else{
				if(goal_index == 2 || goal_index == 11 || goal_index == 20 || goal_index == 29 || goal_index == 38){
                	gripper.clamp();
				}
				
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