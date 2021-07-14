#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_msgs/String.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <queue>
#include <vector>
#define SECURITY_DISTANCE 0.4
#define SECURITY_SPEED 0.4
#define EPISILON 0.03
//lArm lHand lWrist rArm rHand rWrist
/*void* generate_thread(void *arg)
{
    ros::NodeHandle* nh = (ros::NodeHandle*) arg;
    tf::TransformBroadcaster br;
    tf::Transform base_goal;
    while(ros::ok())
    {
    base_goal.setOrigin(tf::Vector3(0.0, 0.65, 0.0));
    tf::Quaternion qq;
    qq.setRPY(0,0,0);
    base_goal.setRotation(qq);
    br.sendTransform(tf::StampedTransform(base_goal, ros::Time::now(), "marker_0", "base"));
    }
}*/
using std::queue;
using std::vector;

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
int main(int argc, char** argv)
{

	ros::init(argc, argv, "generate_goal");
	ros::NodeHandle nh;
	tf::TransformListener tf_linstener;




	tf::TransformBroadcaster br;
	tf::StampedTransform tool_obstacle;
	tf::StampedTransform base_obstacle;
	tf::StampedTransform tool_rHand;
	tf::StampedTransform tool_rArm;
	tf::StampedTransform tool_rWrist;
	tf::StampedTransform tool_lHand;
	tf::StampedTransform tool_lArm;
	tf::StampedTransform tool_lWrist;

	//tf::StampedTransform judge_move;
	tf::StampedTransform base_att;

	tf::StampedTransform test;
	tf::StampedTransform base_tool;

	//pthread_t start_base;
   // pthread_create(&start_base, NULL, generate_thread, (void*)&nh);

	vector<tf::Transform> att_goals;
	tf::Transform att_goal;
	tf::Transform att_goal_two;
	tf::Transform att_goal_middle;
	tf::Transform repu_goal;
	tf::StampedTransform attraction;
	tf::StampedTransform repulsive;
	tf::Transform goal;

	//att_goal.setOrigin(tf::Vector3(0.31174, -0.703138, 0.160335));
	//att_goal_two.setOrigin(tf::Vector3(-0.05, -0.2, 0.66));
	att_goal.setOrigin(tf::Vector3(-0.4, -0.4, 0.25));
	att_goal_two.setOrigin(tf::Vector3(0.4, -0.4, 0.25));
	att_goal_middle.setOrigin(tf::Vector3(0.0, -0.5, 0.25));
	tf::Quaternion q(0.375, 0.925, -0.011, 0.062);
	q = q.normalize();
	//q.setRPY(0.0, 0.0, 0.0);
	att_goal.setRotation(q);
	att_goal_middle.setRotation(q);
	att_goal_two.setRotation(q);
	std::string att_goal_frame;
	att_goals.push_back(att_goal);
	att_goals.push_back(att_goal_middle);
	att_goals.push_back(att_goal_two);
	queue<int> target_queue;
	//target_queue.push(1);
	target_queue.push(2);
	//target_queue.push(1);
	target_queue.push(0);
	int goal_index = target_queue.front();

	target_queue.pop();
	target_queue.push(goal_index);

	tf::Vector3 oposition;
	tf::Vector3 nopsition;
	tf::Vector3 n_o_distance;
	// while(ros::ok()){

	// 	try{
	// 		std::cout <<"wait for connecting"<<std::endl;
	// 		//tf_linstener.waitForTransform("tool0", "human_0/rWrist", ros::Time(0), ros::Duration(3.0));
	// 		//tf_linstener.lookupTransform("tool0", "human_0/rWrist", ros::Time(0), tool_obstacle);
	// 		std::cout << tool_obstacle.getRotation().getX() << std::endl;
	// 		std::cout << tool_obstacle.getRotation().getY() << std::endl;
	// 		std::cout << tool_obstacle.getRotation().getZ() << std::endl;
	// 		std::cout << tool_obstacle.getRotation().getW() << std::endl;
	// 		break;
	// 	}
	// 	catch(tf::TransformException ex){
	// 		ROS_ERROR(ex.what());
	// 		continue;
	// 	}
	// }
	ros::Rate rate(60);
	while(ros::ok())
	{

		// test = tool_obstacle;
		//nh.param<std::string>("att_goal_frame", att_goal_frame, "marker_0");
		br.sendTransform(tf::StampedTransform(att_goals[goal_index], ros::Time::now(), "base", "rect_goal"));


		try{
			//lArm lHand lWrist rArm rHand rWrist
			// tf_linstener.waitForTransform("tool0", "human_0/rWrist", ros::Time(0), ros::Duration(3.0));
			// tf_linstener.waitForTransform("tool0", "human_0/rElbow", ros::Time(0), ros::Duration(3.0));
			// tf_linstener.waitForTransform("tool0", "human_0/rShoulder", ros::Time(0), ros::Duration(3.0));
			// tf_linstener.waitForTransform("tool0", "human_0/lWrist", ros::Time(0), ros::Duration(3.0));
			// tf_linstener.waitForTransform("tool0", "human_0/lElbow", ros::Time(0), ros::Duration(3.0));
			// tf_linstener.waitForTransform("tool0", "human_0/lShoulder", ros::Time(0), ros::Duration(3.0));
			//tf_linstener.waitForTransform("base", "human_1/rHand", ros::Time(0), ros::Duration(3.0));//need
			tf_linstener.waitForTransform("base", "tool0", ros::Time(0), ros::Duration(3.0));//need

			// tf_linstener.lookupTransform("tool0", "human_0/rWrist", ros::Time(0), tool_rHand);
			// tf_linstener.lookupTransform("tool0", "human_0/rElbow", ros::Time(0), tool_rArm);
			// tf_linstener.lookupTransform("tool0", "human_0/rShoulder", ros::Time(0), tool_rWrist);
			// tf_linstener.lookupTransform("tool0", "human_0/lWrist", ros::Time(0), tool_lHand);
			// tf_linstener.lookupTransform("tool0", "human_0/lElbow", ros::Time(0), tool_lArm);
			// tf_linstener.lookupTransform("tool0", "human_0/lShoulder", ros::Time(0), tool_lWrist);
			//tf_linstener.lookupTransform("tool0", "human_1/rHand", ros::Time(0), tool_obstacle);//need
			//tf_linstener.lookupTransform("base", "human_1/rHand", ros::Time(0), base_obstacle);//need
			tf_linstener.lookupTransform("base", "tool0", ros::Time(0), base_tool);//need

		}
		catch(tf::TransformException ex){
			continue;
		}
		//std::cout<<"arr[0]:"<<tool_rHand.getOrigin().length()<<std::endl;
		/*std::cout<<"arr[1]:"<<tool_rArm.getOrigin().length()<<std::endl;
		std::cout<<"arr[2]:"<<tool_rWrist.getOrigin().length()<<std::endl;
		std::cout<<"arr[3]:"<<tool_lHand.getOrigin().length()<<std::endl;
		std::cout<<"arr[4]:"<<tool_lArm.getOrigin().length()<<std::endl;
		std::cout<<"arr[5]:"<<tool_rWrist.getOrigin().length()<<std::endl;*/
	// 	int obstacle_joint = judgejoint(tool_rHand,tool_rArm,tool_rWrist,tool_lHand,tool_lArm,tool_lWrist);
	// 	std::cout<< obstacle_joint<<std::endl;
	// 	switch(obstacle_joint)
	// 	{
	// 		case 0:
	// 			try{
	// 				tf_linstener.waitForTransform("base", "human_0/rWrist", ros::Time(0), ros::Duration(3.0));
	// 				tf_linstener.lookupTransform("base", "human_0/rWrist", ros::Time(0), base_obstacle);
	// 			}
	// 			catch(tf::TransformException ex){
	// 				continue;
	// 		}
	// 			tool_obstacle = tool_rHand;
	// 			break;
	// 		case 1:
	// 			try{
	// 				tf_linstener.waitForTransform("base", "human_0/rElbow", ros::Time(0), ros::Duration(3.0));
	// 				tf_linstener.lookupTransform("base", "human_0/rElbow", ros::Time(0), base_obstacle);
	// 			}
	// 			catch(tf::TransformException ex){
	// 				continue;
	// 			}
	// 			tool_obstacle = tool_rArm;
	// 			break;
	// 		case 2:
	// 			try{
	// 				tf_linstener.waitForTransform("base", "human_0/rShoulder", ros::Time(0), ros::Duration(3.0));
	// 				tf_linstener.lookupTransform("base", "human_0/rShoulder", ros::Time(0), base_obstacle);
	// 			}
	// 			catch(tf::TransformException ex){
	// 				continue;
	// 			}
	// 			tool_obstacle = tool_rWrist;
	// 			break;
	// 		case 3:
	// 			try{
	// 				tf_linstener.waitForTransform("base", "human_0/lWrist", ros::Time(0), ros::Duration(3.0));
	// 				tf_linstener.lookupTransform("base", "human_0/lWrist", ros::Time(0), base_obstacle);
	// 			}
	// 			catch(tf::TransformException ex){
	// 				continue;
	// 			}
	// 			tool_obstacle = tool_lHand;
	// 			break;
	// 		case 4:
	// 			try{
	// 				tf_linstener.waitForTransform("base", "human_0/lElbow", ros::Time(0), ros::Duration(3.0));
	// 				tf_linstener.lookupTransform("base", "human_0/lElbow", ros::Time(0), base_obstacle);
	// 			}
	// 			catch(tf::TransformException ex){
	// 				continue;
	// 			}
	// 			tool_obstacle = tool_lArm;
	// 			break;
	// 		case 5:
	// 			try{
	// 				tf_linstener.waitForTransform("base", "human_0/lShoulder", ros::Time(0), ros::Duration(3.0));
	// 				tf_linstener.lookupTransform("base", "human_0/lShoulder", ros::Time(0), base_obstacle);
	// 			}
	// 			catch(tf::TransformException ex){
	// 				continue;
	// 			}
	// 			tool_obstacle = tool_lWrist;
	// 			break;
	// 		default:
	// 			std::cout<<"oh, shit"<<std::endl;

	// 	}
	// 	std::cout << tool_obstacle.getOrigin().length() << std::endl;
	// 	tf::Vector3 obstacle_tool = base_tool.getOrigin()-base_obstacle.getOrigin();
	// 	tf::Vector3 distance = tool_obstacle.getOrigin();
	// 	double tool0_obstacle_distance = distance.length();
	// 	oposition = tf::Vector3(test.getOrigin().x(), test.getOrigin().y(), test.getOrigin().z());
	// 	nopsition = tf::Vector3(tool_obstacle.getOrigin().x(), tool_obstacle.getOrigin().y(), tool_obstacle.getOrigin().z());
	// 	n_o_distance = nopsition - oposition;
	// 	//std::cout << "n_o_oposition:" <<  n_o_distance.length() << std::endl;
	// 	double obstacle_speed = n_o_distance.length()*60;
	// 	//std::cout << "speed:" <<  obstacle_speed << std::endl;
	// //	std::cout << "distance:" <<  distance.length() << std::endl;
	// 	if(distance.length() > SECURITY_DISTANCE)
	// 	{
	// 		nh.setParam("avoid", false);
	// 		repu_goal.setOrigin(tf::Vector3(base_tool.getOrigin().x(), base_tool.getOrigin().y(), base_tool.getOrigin().z()));
	// 		repu_goal.setRotation(q);
	// 		br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
	// 	}
	// 	else if(distance.length() < SECURITY_DISTANCE)
	// 	{
	// 		nh.setParam("avoid",true);
	// 		if(obstacle_speed < SECURITY_SPEED)
	// 		{
	// 			float x = obstacle_tool.x()*1/((exp(5*distance.length()))*distance.length()) + base_tool.getOrigin().x();
	// 			float y = obstacle_tool.y()*1/((exp(5*distance.length()))*distance.length()) + base_tool.getOrigin().y();
	// 			float z = obstacle_tool.z()*1/((exp(5*distance.length()))*distance.length()) + base_tool.getOrigin().z();
	// 			repu_goal.setOrigin(tf::Vector3(x, y, z));
	// 			repu_goal.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
	// 			br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
	// 		}
	// 		else if(obstacle_speed > SECURITY_SPEED)
	// 		{
	// 			if(oposition.length() > nopsition.length()){
	// 				float a = obstacle_tool.x()*1/((exp(5*distance.length()))*distance.length());
	// 				float b = obstacle_tool.y()*1/((exp(5*distance.length()))*distance.length());
	// 				float c = obstacle_tool.z()*1/((exp(5*distance.length()))*distance.length());
	// 				tf::Vector3 abc = tf::Vector3(a,b,c);
	// 				float aa = -(b+c)/a;
	// 				float bb = 1;
	// 				float cc = 1;
	// 				tf::Vector3 xyz = tf::Vector3(aa,bb,cc);
	// 				float x = aa*abc.length()/xyz.length() + base_tool.getOrigin().x();
	// 				float y = bb*abc.length()/xyz.length() + base_tool.getOrigin().y();
	// 				float z = cc*abc.length()/xyz.length() + base_tool.getOrigin().z();
	// 				repu_goal.setOrigin(tf::Vector3(x,y,z));
	// 				repu_goal.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	// 				br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
	// 			}
	// 			else if(oposition.length() < nopsition.length()){
	// 				repu_goal.setOrigin(tf::Vector3(base_tool.getOrigin().x(), base_tool.getOrigin().y(), base_tool.getOrigin().z()));
	// 				repu_goal.setRotation(q);
	// 				br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
	// 			}

	// 		}
	// 	}

	// 	try{
	// 		tf_linstener.lookupTransform("tool0", "attraction_goal", ros::Time(0), attraction);
	// 		tf_linstener.lookupTransform("tool0", "repulsiv_goal", ros::Time(0), repulsive);
	// 	//	tf_linstener.lookupTransform("base", "attraction_goal", ros::Time(0), base_att);
	// 	}
	// 	catch(tf::TransformException ex){
	// 		continue;
	// 	}
	// 	//tf::Vector3 new_attraction = base_att.getOrigin();
	// 	//std::cout << "ATTRACTION:" <<new_attraction.x()<< std::endl;
	// 	//std::cout << "ATTRACTION:" <<new_attraction.y()<< std::endl;
	// 	//std::cout << "ATTRACTION:" <<new_attraction.z()<< std::endl;
	// 	//std::cout << "ATTRACTION:" <<base_att.getRotation().getZ()<< std::endl;
	// 	//tf::Vector3 re_tool = repulsive.getOrigin();
	// 	//std::cout << "repu_goal" << re_tool.length() << std::endl;
	// 	goal.setOrigin(tf::Vector3(attraction.getOrigin().x() + repulsive.getOrigin().x(),attraction.getOrigin().y() + repulsive.getOrigin().y(),attraction.getOrigin().z() + repulsive.getOrigin().z()));
	// 	//goal.setOrigin(tf::Vector3(repulsive.getOrigin().x(),repulsive.getOrigin().y(),repulsive.getOrigin().z()));
	// 	goal.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	// 	br.sendTransform(tf::StampedTransform(goal, ros::Time::now(), "tool0", "desired_goal"));

		if(fabs(base_tool.getOrigin().x()-att_goals[goal_index].getOrigin().x()) < 0.03 && fabs(base_tool.getOrigin().y() -att_goals[goal_index].getOrigin().y() ) < 0.03 && fabs(base_tool.getOrigin().z() - att_goals[goal_index].getOrigin().z()) < 0.03)
		{
			goal_index = target_queue.front();
			target_queue.pop();
			target_queue.push(goal_index);
		}
		rate.sleep();
	}
	return 0;
}