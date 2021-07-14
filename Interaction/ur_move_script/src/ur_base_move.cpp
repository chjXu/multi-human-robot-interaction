#include "ur_move_script/ur_base_move.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ur_base");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    UR ur1;
    ROS_INFO("Starting...");
    while (ros::ok())
    {   
        ur1.updata();
        ur1.pub();
        ros::spinOnce();
        rate.sleep();
    }
}