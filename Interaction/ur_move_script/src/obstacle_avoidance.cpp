#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_msgs/String.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <queue>
#include <string>
#include <vector>
#define SECURITY_DISTANCE 0.4
#define SECURITY_SPEED 0.4
#define EPISILON 0.03

using namespace std;
using std::queue;
using std::vector;


class Goals{
public:
    Goals(const string& name){
        this->name = name;
        setJointName();
        setJoint();
        q.setRPY(0,0,0);
    }
    tf::StampedTransform get_tool_obastacle(){
        return tool_obstacle;
    }
    void setQ(double x, double y, double z, double w);
    void setJointName();
    void setJoint();
    bool init();
    bool getAllJointTrans();
    int judgejoint(tf::StampedTransform& , tf::StampedTransform& , tf::StampedTransform& , tf::StampedTransform& , tf::StampedTransform& , tf::StampedTransform& );
    void switchJoint();
    void getPower();
    void getAttAndRepu(tf::StampedTransform &attraction, tf::StampedTransform &repulsive, tf::StampedTransform &base_tool);
private:
    ros::NodeHandle nh;
    string name;
    vector<string> joint_name;
    vector<tf::StampedTransform> joint_obstacle;

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster br;
    tf::Quaternion q;

    tf::StampedTransform test;
    tf::StampedTransform base_tool;
    tf::StampedTransform tool_obstacle;
	tf::StampedTransform base_obstacle;
	tf::StampedTransform tool_rHand;
	tf::StampedTransform tool_rArm;
	tf::StampedTransform tool_rWrist;
	tf::StampedTransform tool_lHand;
	tf::StampedTransform tool_lArm;
	tf::StampedTransform tool_lWrist;

    tf::Transform repu_goal;
    tf::StampedTransform attraction;
    tf::StampedTransform repulsive;

    tf::Vector3 oposition;
   	tf::Vector3 nopsition;
   	tf::Vector3 n_o_distance;
};

void Goals::setQ(double x, double y, double z, double w){}

void Goals::getAttAndRepu(tf::StampedTransform &attraction, tf::StampedTransform &repulsive, tf::StampedTransform &base_tool){
    attraction = this->attraction;
    repulsive = this->repulsive;
    base_tool = this->base_tool;
}

void Goals::setJoint(){
    joint_obstacle.emplace_back(tool_rHand);
    // joint_obstacle.emplace_back(tool_rArm);
    // joint_obstacle.emplace_back(tool_rWrist);
    // joint_obstacle.emplace_back(tool_lHand);
    // joint_obstacle.emplace_back(tool_lArm);
    // joint_obstacle.emplace_back(tool_lWrist);
}

void Goals::setJointName(){
    string rHand = name + "/rHand";
    // string rArm = name + "/rArm";
    // string rWrist = name + "/rWrist";
    // string lShoulder = name + "/lShoulder";
    // string lArm = name + "/lArm";
    // string lWrist = name + "/lWrist";
    joint_name.push_back(rHand);
    // joint_name.push_back(rArm);
    // joint_name.push_back(rWrist);
    // joint_name.push_back(lShoulder);
    // joint_name.push_back(lArm);
    // joint_name.push_back(lWrist);
}

bool Goals::init(){
    
}

bool Goals::getAllJointTrans(){
    test = tool_obstacle;
    try{
        //cout << joint_name.size() << endl;
        for(int i=0; i<joint_name.size(); i++){
            tf_listener.waitForTransform("tool0", joint_name[i], ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("tool0", joint_name[i], ros::Time(0), joint_obstacle[i]);
        }
        tf_listener.waitForTransform("base", "tool0", ros::Time(0), ros::Duration(3.0));//need
        tf_listener.lookupTransform("base", "tool0", ros::Time(0), base_tool);//need
    }
    catch(tf::TransformException ex){
        ROS_ERROR(ex.what());
    }

    return true;
}

int Goals::judgejoint(tf::StampedTransform& one_joint, tf::StampedTransform& two_joint, tf::StampedTransform& three_joint, tf::StampedTransform& four_joint, tf::StampedTransform& five_joint, tf::StampedTransform& six_joint)
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

void Goals::switchJoint(){
    //int obstacle_joint = judgejoint(joint_obstacle[0], joint_obstacle[1], joint_obstacle[2], joint_obstacle[3], joint_obstacle[4], joint_obstacle[5]);
    //std::cout<< obstacle_joint<<std::endl;
    while(ros::ok()){
        try{
            tf_listener.waitForTransform("base", joint_name[0], ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("base", joint_name[0], ros::Time(0), base_obstacle);
        }
        catch(tf::TransformException ex){
            continue;
        }
        tool_obstacle = joint_obstacle[0];
        break;
    }
    std::cout << tool_obstacle.getOrigin().length() << std::endl;
}

void Goals::getPower(){
    tf::Vector3 obstacle_tool = base_tool.getOrigin()-base_obstacle.getOrigin();
    tf::Vector3 distance = tool_obstacle.getOrigin();

    double tool0_obstacle_distance = distance.length();
    oposition = tf::Vector3(test.getOrigin().x(), test.getOrigin().y(), test.getOrigin().z());
    nopsition = tf::Vector3(tool_obstacle.getOrigin().x(), tool_obstacle.getOrigin().y(), tool_obstacle.getOrigin().z());
    n_o_distance = nopsition - oposition;

    double obstacle_speed = n_o_distance.length()*60;

    if(distance.length() > SECURITY_DISTANCE)
    {
        nh.setParam("avoid", false);
        repu_goal.setOrigin(tf::Vector3(base_tool.getOrigin().x(), base_tool.getOrigin().y(), base_tool.getOrigin().z()));
        repu_goal.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
        br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
    }
    else if(distance.length() < SECURITY_DISTANCE)
    {
        nh.setParam("avoid",true);
        if(obstacle_speed < SECURITY_SPEED)
        {
            float x = obstacle_tool.x()*1/((exp(5*distance.length()))*distance.length()) + base_tool.getOrigin().x();
            float y = obstacle_tool.y()*1/((exp(5*distance.length()))*distance.length()) + base_tool.getOrigin().y();
            float z = obstacle_tool.z()*1/((exp(5*distance.length()))*distance.length()) + base_tool.getOrigin().z();
            repu_goal.setOrigin(tf::Vector3(x, y, z));
            repu_goal.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
            br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
        }
        else if(obstacle_speed > SECURITY_SPEED)
        {
            if(oposition.length() > nopsition.length()){
                float a = obstacle_tool.x()*1/((exp(5*distance.length()))*distance.length());
                float b = obstacle_tool.y()*1/((exp(5*distance.length()))*distance.length());
                float c = obstacle_tool.z()*1/((exp(5*distance.length()))*distance.length());
                tf::Vector3 abc = tf::Vector3(a,b,c);
                float aa = -(b+c)/a;
                float bb = 1;
                float cc = 1;
                tf::Vector3 xyz = tf::Vector3(aa,bb,cc);
                float x = aa*abc.length()/xyz.length() + base_tool.getOrigin().x();
                float y = bb*abc.length()/xyz.length() + base_tool.getOrigin().y();
                float z = cc*abc.length()/xyz.length() + base_tool.getOrigin().z();
                repu_goal.setOrigin(tf::Vector3(x,y,z));
                repu_goal.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
                br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
            }
            else if(oposition.length() < nopsition.length()){
                repu_goal.setOrigin(tf::Vector3(base_tool.getOrigin().x(), base_tool.getOrigin().y(), base_tool.getOrigin().z()));
                repu_goal.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
                br.sendTransform(tf::StampedTransform(repu_goal, ros::Time::now(), "base", "repulsiv_goal"));
            }

        }
    }

    try{
        tf_listener.lookupTransform("tool0", "attraction_goal", ros::Time(0), attraction);
        tf_listener.lookupTransform("tool0", "repulsiv_goal", ros::Time(0), repulsive);
    //	tf_linstener.lookupTransform("base", "attraction_goal", ros::Time(0), base_att);
    }
    catch(tf::TransformException ex){
        ROS_ERROR(ex.what());
    }

}

void* human2_thread(void *arg){
    Goals human2("human_1");
    tf::TransformBroadcaster br;
    tf::StampedTransform attraction_2;
    tf::StampedTransform repulsive_2;
    tf::StampedTransform base_tool;

    while (ros::ok()) {
        if(human2.init()){
            break;
        }
    }

    ros::Rate rate(60);
    while (ros::ok()) {
        if(human2.getAllJointTrans()){
            human2.switchJoint();
            human2.getPower();
            human2.getAttAndRepu(attraction_2, repulsive_2, base_tool);
        }
        br.sendTransform(tf::StampedTransform(repulsive_2, ros::Time::now(), "base", "repulsive_2"));

        ros::spinOnce();
        rate.sleep();
    }

}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "obs_avoidance");
   tf::TransformBroadcaster br;
   tf::TransformListener tf_listener;
   vector<tf::Transform> att_goals;
   tf::Transform att_goal;
   tf::Transform att_goal_two;
   tf::Transform att_goal_middle;
   tf::Transform goal;
   tf::StampedTransform attraction;
   tf::StampedTransform repulsive;
   tf::StampedTransform repulsive_2;
   tf::StampedTransform base_tool;
   tf::StampedTransform test;

   att_goal.setOrigin(tf::Vector3(-0.4, -0.4, 0.3));
   att_goal_two.setOrigin(tf::Vector3(0.4, -0.4, 0.3));
   att_goal_middle.setOrigin(tf::Vector3(0.0, -0.6, 0.3));
   tf::Quaternion q(0.375, 0.925, -0.011, 0.062);
   q = q.normalize();
   //q.setRPY(0.0, 0.0, 0.0);
   att_goal.setRotation(q);
   att_goal_middle.setRotation(q);
   att_goal_two.setRotation(q);

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

   Goals human1("human_0");
   bool check = false;

   pthread_t human_id;
   pthread_create(&human_id, NULL, human2_thread, NULL);

   //human("lhz");
	while(ros::ok()){
        if(human1.init()){
            break;
        }
	}
	ros::Rate rate(60);
	while(ros::ok())
	{
        //test =  human1.get_tool_obastacle();
        br.sendTransform(tf::StampedTransform(att_goals[goal_index], ros::Time::now(), "base", "attraction_goal"));
        if(human1.getAllJointTrans()){
            human1.switchJoint();
            human1.getPower();
            human1.getAttAndRepu(attraction, repulsive, base_tool);
        }

        try{
            tf_listener.lookupTransform("base", "repulsive_2", ros::Time(0), repulsive_2);
            check = true;
        }
        catch(tf::TransformException ex){
            ROS_ERROR(ex.what());
            cout << "Only one human!" << endl;
        }

        if(check){
            goal.setOrigin(tf::Vector3(attraction.getOrigin().x() + repulsive.getOrigin().x() + repulsive_2.getOrigin().x(),attraction.getOrigin().y() + repulsive.getOrigin().y() + repulsive_2.getOrigin().y(),attraction.getOrigin().z() + repulsive.getOrigin().z() + repulsive_2.getOrigin().z()));
        }else{
		    goal.setOrigin(tf::Vector3(attraction.getOrigin().x() + repulsive.getOrigin().x(),attraction.getOrigin().y() + repulsive.getOrigin().y(),attraction.getOrigin().z() + repulsive.getOrigin().z()));
        }
        //goal.setOrigin(tf::Vector3(repulsive.getOrigin().x(),repulsive.getOrigin().y(),repulsive.getOrigin().z()));
		goal.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
		br.sendTransform(tf::StampedTransform(goal, ros::Time::now(), "tool0", "desired_goal"));

		if(fabs(base_tool.getOrigin().x()-att_goals[goal_index].getOrigin().x()) < 0.03 && fabs(base_tool.getOrigin().y() -att_goals[goal_index].getOrigin().y() ) < 0.03 && fabs(base_tool.getOrigin().z() - att_goals[goal_index].getOrigin().z()) < 0.03)
		{
			goal_index = target_queue.front();
			target_queue.pop();
			target_queue.push(goal_index);
		}
        ros::spinOnce();
		rate.sleep();
	}
	return 0;
}