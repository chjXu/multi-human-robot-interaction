#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_msgs/String.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <queue>
#include <vector>
#include <fstream>
#include <chrono>
#define SECURITY_DISTANCE 0.4
#define SECURITY_SPEED 0.4
#define EPISILON 0.03

using std::queue;
using std::vector;
using namespace std;


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "sin_goal");
	ros::NodeHandle nh;
	tf::TransformListener tf_linstener;

	tf::TransformBroadcaster br;
    tf::StampedTransform base_tool;
    tf::StampedTransform base_obs;
	
    double init_pose[3] = {0.0, -0.5, 0.3};


	tf::Quaternion q(0.965, -0.258, -0.003, -0.015);
	q = q.normalize();

    std::string record_dir = "/home/xuchengjun/open_ros_codegen/data/end.txt";
    //std::string record_dir_2 = "/home/xuchengjun/open_ros_codegen/data/ee_track.txt";
    bool record = true;
    char* write_buffer = NULL;
    std::ofstream fs;
    if(record){
        fs.open(record_dir.c_str());
        if(fs){
            //0fs << "x \n";
        }
        write_buffer = (char*)malloc(256);
    }

	while(ros::ok()){

		try{
			// std::cout <<"wait for connecting"<<std::endl;
			// tf_linstener.waitForTransform("tool0", "base", ros::Time(0), ros::Duration(3.0));
			// tf_linstener.lookupTransform("tool0", "base", ros::Time(0), tool_obstacle);
			// std::cout << tool_obstacle.getOrigin().length() << std::endl;
			break;
		}
		catch(tf::TransformException ex){
			ROS_ERROR(ex.what());
			continue;
		}
	}
	ros::Rate rate(15);
    double periord = 1.0;
    int init_angle = 0;

	while(ros::ok())
	{	
        //auto start = chrono::system_clock::now();
        double radius = 0.4;
        //periord += 0.2;
        init_angle += 2;
        double angle = M_PI * ((double)init_angle / 360);
		// if(angle == 180){
		// 	sleep(1000);
		// }
		// if(angle == 360){
		// 	init_angle = 0.0;
		// 	sleep(1000);
		// }
        double delta_x = radius * std::sin(angle);
        double x = init_pose[0] + delta_x;

        //cout << x << endl;

        tf::Transform sin_goal;
        sin_goal.setOrigin(tf::Vector3(x, init_pose[1], init_pose[2]));
        sin_goal.setRotation(q);

		br.sendTransform(tf::StampedTransform(sin_goal, ros::Time::now(), "base", "rect_goal"));
	
		try{
			
			// tf_linstener.waitForTransform("base", "tool0", ros::Time(0), ros::Duration(3.0));//need
			tf_linstener.lookupTransform("base", "tool0", ros::Time(0), base_tool);//need
            cout << "Connected..." << endl;
			
			double track = base_tool.getOrigin().getX();
        	if(record && fs){
            	sprintf(write_buffer, "%.4f, %.4f\n", x, track);
            	fs.write(write_buffer, strlen(write_buffer));
        	}

		}
		catch(tf::TransformException ex){
			continue;
		}



        // auto end = chrono::system_clock::now();
        // auto duration = chrono::duration_cast<chrono::microseconds>(end-start);
        // cout << double(duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den << endl;

		// while (ros::ok())
		// {
		// 	try
		// 	{
		// 		tf_linstener.lookupTransform("base", "tool0", ros::Time(0), base_tool);
		// 		tf_linstener.lookupTransform("base", "marker", ros::Time(0), base_obs);
		// 	}
		// 	catch(const std::exception& e)
		// 	{
		// 		std::cerr << e.what() << '\n';
		// 	}

        //     if(fabs(base_tool.getOrigin().x()-base_obs.getOrigin().x()) < 0.03 && 
        //        fabs(base_tool.getOrigin().y() -base_obs.getOrigin().y() ) < 0.03 && 
        //        fabs(base_tool.getOrigin().z() -base_obs.getOrigin().z()) < 0.03)
		//     {
        //         break;
		//     }
    	// }


		rate.sleep();
	}
	return 0;
}